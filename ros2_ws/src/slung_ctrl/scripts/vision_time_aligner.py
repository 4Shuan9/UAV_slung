#!/usr/bin/env python3
# ==============================================================================
# @File  : vision_time_aligner.py
# @Brief : Convert Gazebo image/swing timestamps from simulation time to the
#          existing ROS/system-time domain without changing /clock or use_sim_time.
#
# Runtime role:
#   Gazebo image acquisition stamp (sim time)
#       -> raw visual swing angle
#       -> this adapter maps stamp into system time
#       -> aligned visual swing angle used by swing_angle_eso_fusion
#
# This node intentionally publishes only the aligned measurement. Delay age is
# computed inside the estimator from the aligned timestamp, so no extra debug
# delay topic is required in the normal runtime graph.
# ==============================================================================

import threading
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from builtin_interfaces.msg import Time

from gz.msgs10.clock_pb2 import Clock
from gz.transport13 import Node as GzNode


GZ_CLOCK_TOPIC = "/clock"
RAW_VISION_TOPIC = "/uav/cv/swing_angle/world_frame/cartesian"
ALIGNED_VISION_TOPIC = "/uav/cv/swing_angle_aligned/world_frame/cartesian"
MAX_CLOCK_SAMPLES = 20000


class VisionTimeAligner(Node):
    def __init__(self):
        super().__init__("vision_time_aligner")

        self._lock = threading.Lock()
        self._clock_samples = deque(maxlen=MAX_CLOCK_SAMPLES)
        self._last_sim_ns = None
        self._invalid_count = 0

        # Read Gazebo's native clock directly. We deliberately do NOT bridge
        # /clock into ROS and do NOT switch any node to use_sim_time.
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

        self.get_logger().info(
            "VisionTimeAligner started. Existing clock configuration is unchanged.\n"
            f"  Raw vision : {RAW_VISION_TOPIC}\n"
            f"  Aligned    : {ALIGNED_VISION_TOPIC}"
        )

    @staticmethod
    def _pair_to_ns(sec, nsec):
        return int(sec) * 1_000_000_000 + int(nsec)

    @staticmethod
    def _ns_to_ros_time(stamp_ns):
        stamp_ns = int(stamp_ns)
        return Time(
            sec=stamp_ns // 1_000_000_000,
            nanosec=stamp_ns % 1_000_000_000,
        )

    def _clock_callback(self, msg):
        sim_ns = self._pair_to_ns(msg.sim.sec, msg.sim.nsec)
        system_ns = self._pair_to_ns(msg.system.sec, msg.system.nsec)

        with self._lock:
            # Gazebo restart / world reset: never mix two simulation epochs.
            if self._last_sim_ns is not None and sim_ns < self._last_sim_ns:
                self._clock_samples.clear()
                self._invalid_count = 0
                self.get_logger().warning(
                    "Gazebo simulation time moved backwards; alignment history reset."
                )

            self._last_sim_ns = sim_ns
            self._clock_samples.append((sim_ns, system_ns))

    def _sim_to_system_ns(self, target_sim_ns):
        """Map one Gazebo sim timestamp to system time by local interpolation."""
        with self._lock:
            samples = list(self._clock_samples)

        if len(samples) < 2:
            return None

        # Visual latency is only tens of milliseconds, so the correct bracket
        # should normally be close to the newest /clock samples.
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
        image_sim_ns = self._pair_to_ns(
            msg.header.stamp.sec,
            msg.header.stamp.nanosec,
        )

        image_system_ns = self._sim_to_system_ns(image_sim_ns)
        if image_system_ns is None:
            self._invalid_count += 1
            if self._invalid_count == 1 or self._invalid_count % 25 == 0:
                self.get_logger().warning(
                    "No matching Gazebo /clock history for current visual frame; frame skipped."
                )
            return

        aligned = Vector3Stamped()
        aligned.header.stamp = self._ns_to_ros_time(image_system_ns)
        aligned.header.frame_id = msg.header.frame_id
        aligned.vector.x = msg.vector.x
        aligned.vector.y = msg.vector.y
        aligned.vector.z = msg.vector.z
        self._aligned_pub.publish(aligned)


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
