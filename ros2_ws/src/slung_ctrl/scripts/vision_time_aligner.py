#!/usr/bin/env python3
# ==============================================================================
# @File     : vision_time_aligner.py
# @Brief    : Gazebo sim-time -> ROS system-time alignment for visual swing data.
#
# Stage-1C role:
#   1) Subscribe directly to Gazebo /clock (sim + system time in one message).
#   2) Subscribe to the existing raw visual swing-angle topic whose header stamp
#      is the original Gazebo image acquisition sim-time.
#   3) Map the acquisition stamp into ROS/system-time without changing any
#      existing topic or clock configuration.
#   4) Publish an aligned copy and the measured end-to-end visual delay.
#
# IMPORTANT:
#   - This node is diagnostic / alignment only.
#   - It does NOT modify /clock, use_sim_time, PX4 timestamps, the original
#     visual topic, PESO/LESO state, or ADRC control.
# ==============================================================================

import threading
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float64
from builtin_interfaces.msg import Time

from gz.msgs10.clock_pb2 import Clock
from gz.transport13 import Node as GzNode


GZ_CLOCK_TOPIC = "/clock"
RAW_VISION_TOPIC = "/uav/cv/swing_angle/world_frame/cartesian"
ALIGNED_VISION_TOPIC = "/uav/cv/swing_angle_aligned/world_frame/cartesian"
DELAY_TOPIC = "/uav/cv/vision_delay_ms"

# Gazebo /clock is high-rate. This is intentionally much longer than the
# tens-of-milliseconds visual latency measured in Stage 1B.
MAX_CLOCK_SAMPLES = 20000

# Sanity guard only. Values outside this range are not published.
MIN_VALID_DELAY_MS = -5.0
MAX_VALID_DELAY_MS = 1000.0


class VisionTimeAligner(Node):
    def __init__(self):
        super().__init__("vision_time_aligner")

        self._lock = threading.Lock()
        self._clock_samples = deque(maxlen=MAX_CLOCK_SAMPLES)
        self._last_sim_ns = None

        self._valid_count = 0
        self._invalid_count = 0
        self._delay_sum_ms = 0.0
        self._delay_min_ms = float("inf")
        self._delay_max_ms = float("-inf")

        # Gazebo transport subscription. We deliberately do NOT bridge /clock
        # into ROS and do NOT switch any node to use_sim_time.
        self._gz_node = GzNode()
        ok = self._gz_node.subscribe(Clock, GZ_CLOCK_TOPIC, self._clock_callback)
        if not ok:
            raise RuntimeError(f"Failed to subscribe Gazebo topic: {GZ_CLOCK_TOPIC}")

        self._vision_sub = self.create_subscription(
            Vector3Stamped,
            RAW_VISION_TOPIC,
            self._vision_callback,
            10,
        )

        self._aligned_pub = self.create_publisher(
            Vector3Stamped,
            ALIGNED_VISION_TOPIC,
            10,
        )

        self._delay_pub = self.create_publisher(
            Float64,
            DELAY_TOPIC,
            10,
        )

        self.get_logger().info(
            "VisionTimeAligner started. Existing clocks/topics are untouched.\n"
            f"  Gazebo clock : {GZ_CLOCK_TOPIC}\n"
            f"  Raw vision   : {RAW_VISION_TOPIC}\n"
            f"  Aligned      : {ALIGNED_VISION_TOPIC}\n"
            f"  Delay        : {DELAY_TOPIC}"
        )

    @staticmethod
    def _clock_pair_to_ns(sec, nsec):
        return int(sec) * 1_000_000_000 + int(nsec)

    @staticmethod
    def _ns_to_ros_time(stamp_ns):
        stamp_ns = int(stamp_ns)
        return Time(
            sec=stamp_ns // 1_000_000_000,
            nanosec=stamp_ns % 1_000_000_000,
        )

    def _clock_callback(self, msg):
        sim_ns = self._clock_pair_to_ns(msg.sim.sec, msg.sim.nsec)
        system_ns = self._clock_pair_to_ns(msg.system.sec, msg.system.nsec)

        with self._lock:
            # Gazebo restart / world reset. Do not mix two simulation epochs.
            if self._last_sim_ns is not None and sim_ns < self._last_sim_ns:
                self._clock_samples.clear()
                self._valid_count = 0
                self._invalid_count = 0
                self._delay_sum_ms = 0.0
                self._delay_min_ms = float("inf")
                self._delay_max_ms = float("-inf")
                self.get_logger().warning(
                    "Gazebo simulation time moved backwards; alignment history reset."
                )

            self._last_sim_ns = sim_ns
            self._clock_samples.append((sim_ns, system_ns))

    def _sim_to_system_ns(self, target_sim_ns):
        """
        Convert one Gazebo simulation timestamp to its corresponding Gazebo
        system timestamp using the two nearest /clock samples.

        We use interpolation rather than a single fixed sim->system offset.
        """
        with self._lock:
            samples = list(self._clock_samples)

        if len(samples) < 2:
            return None

        # Visual measurements are delayed only by tens of milliseconds, so the
        # desired bracket should normally be close to the newest clock sample.
        for i in range(len(samples) - 2, -1, -1):
            sim0, sys0 = samples[i]
            sim1, sys1 = samples[i + 1]

            if sim0 <= target_sim_ns <= sim1:
                if sim1 == sim0:
                    return sys0

                ratio = (target_sim_ns - sim0) / float(sim1 - sim0)
                return int(sys0 + ratio * (sys1 - sys0))

        return None

    def _vision_callback(self, msg):
        # Fusion currently uses ROS system time (use_sim_time == false).
        # Use the same ROS clock domain here for arrival time.
        receive_system_ns = self.get_clock().now().nanoseconds

        image_sim_ns = self._clock_pair_to_ns(
            msg.header.stamp.sec,
            msg.header.stamp.nanosec,
        )

        image_system_ns = self._sim_to_system_ns(image_sim_ns)

        if image_system_ns is None:
            self._invalid_count += 1
            if self._invalid_count == 1 or self._invalid_count % 25 == 0:
                self.get_logger().warning(
                    "No matching Gazebo /clock history for current visual frame; "
                    "frame skipped."
                )
            return

        delay_ms = (receive_system_ns - image_system_ns) / 1e6

        if delay_ms < MIN_VALID_DELAY_MS or delay_ms > MAX_VALID_DELAY_MS:
            self._invalid_count += 1
            self.get_logger().warning(
                f"Invalid mapped visual delay {delay_ms:.3f} ms; frame skipped."
            )
            return

        # Publish an aligned copy. Vector values and frame semantics are
        # deliberately unchanged; only the timestamp domain is converted.
        aligned = Vector3Stamped()
        aligned.header.stamp = self._ns_to_ros_time(image_system_ns)
        aligned.header.frame_id = msg.header.frame_id
        aligned.vector.x = msg.vector.x
        aligned.vector.y = msg.vector.y
        aligned.vector.z = msg.vector.z
        self._aligned_pub.publish(aligned)

        delay_msg = Float64()
        delay_msg.data = float(delay_ms)
        self._delay_pub.publish(delay_msg)

        self._valid_count += 1
        self._delay_sum_ms += delay_ms
        self._delay_min_ms = min(self._delay_min_ms, delay_ms)
        self._delay_max_ms = max(self._delay_max_ms, delay_ms)

        # ~25 Hz vision -> about one concise line per second.
        if self._valid_count % 25 == 0:
            mean_ms = self._delay_sum_ms / self._valid_count
            self.get_logger().info(
                "delay: current=%.3f ms | mean=%.3f ms | min=%.3f ms | "
                "max=%.3f ms | valid=%d | invalid=%d"
                % (
                    delay_ms,
                    mean_ms,
                    self._delay_min_ms,
                    self._delay_max_ms,
                    self._valid_count,
                    self._invalid_count,
                )
            )


def main(args=None):
    rclpy.init(args=args)
    node = VisionTimeAligner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
