#!/usr/bin/env python3

# ==============================================================================
# @File     : payload_detector.py
# @Author   : 4Shuan9
# @Date     : 2026-07-31
# @Brief    : 视觉识别核心节点，用于识别仿真环境中的吊挂物并发布其像素坐标
# 
# @Main Work: 
#   * 节点注册：创建名为 `payload_detector` 的 ROS 2 节点。
#   * 数据输入：通过 gz-transport 订阅 Gazebo 底层相机图像，输入话题为
#             /world/UAV_slung_world/model/x500_mono_cam_down/link/camera_link/sensor/imager/image
#   * 核心处理：利用 OpenCV 对原始图像进行 HSV 颜色阈值分割，提取红色吊挂物的中心像素坐标
#   * 数据输出：将带有时间戳的像素坐标发布至 ROS 2 网络，输出话题为 /uav/vision/payload_pixel (消息类型: PointStamped)
# ==============================================================================

# === [ Includes & Configuration ] ===
import time
import os
import threading
import sys
import cv2
import numpy as np

import rclpy
from rclpy.node import Node as RosNode
from geometry_msgs.msg import PointStamped 
from builtin_interfaces.msg import Time    
from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node as GzNode

CAMERA_TOPIC = "/world/UAV_slung_world/model/x500_mono_cam_down/link/camera_link/sensor/imager/image"
PUBLISH_TOPIC = "/uav/vision/payload_pixel"
SHOW_IMAGE_WINDOW = os.environ.get("SLUNG_SHOW_IMAGE", "1").lower() not in {"0", "false"}
FPS_UPDATE_PERIOD = 0.1

# === [ ROS Node ] ===
class VisionPublisher(RosNode):
    def __init__(self):
        super().__init__('payload_detector')
        self.pub = self.create_publisher(PointStamped, PUBLISH_TOPIC, 10)

# === [ Gazebo Node ] ===
class GzCamRaw:
    def __init__(self, topic_name):
        self._node = GzNode()
        self._node.subscribe(Image, topic_name, self._cb)
        self._raw_msg = None 
        self._condition = threading.Condition()
        self.is_running = True

    def _cb(self, msg) -> None:
        if not self.is_running:
            return
        with self._condition:
            self._raw_msg = msg
            self._condition.notify_all()

    def get_next_image(self, timeout=None):
        with self._condition:
            if self._raw_msg is None:
                self._condition.wait_for(lambda: self._raw_msg is not None or not self.is_running, timeout=timeout)
            if self._raw_msg is None or not self.is_running:
                return None, None 
            msg = self._raw_msg
            self._raw_msg = None
            
        width, height = msg.width, msg.height
        raw_data = msg.data
        
        sec = msg.header.stamp.sec
        nsec = msg.header.stamp.nsec
        stamp = (sec, nsec)
        
        if not raw_data or len(raw_data) % (width * height) != 0:
            return None, stamp
            
        channels = len(raw_data) // (width * height)
        
        try:
            np_image = np.frombuffer(raw_data, dtype=np.uint8).reshape((height, width, channels))
            if channels == 3:
                return cv2.cvtColor(np_image, cv2.COLOR_RGB2BGR), stamp
            elif channels == 4:
                return cv2.cvtColor(np_image, cv2.COLOR_RGBA2BGR), stamp
            return np_image, stamp
        except ValueError:
            return None, stamp

    def cleanup(self):
        self.is_running = False
        with self._condition:
            self._condition.notify_all()
        if SHOW_IMAGE_WINDOW:
            cv2.destroyAllWindows()

# === [ CV Processing ] ===
def process_and_find_red(img):
    height, width = img.shape[:2]
    pos = None
    
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255]))
    mask2 = cv2.inRange(hsv, np.array([160, 100, 100]), np.array([180, 255, 255]))
    mask = cv2.bitwise_or(mask1, mask2)
    # Preserve the observed outer boundary: opening can split an occluded ball
    # or change the support points of its enclosing circle.
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest_contour) > 30:
            # Codex-generated update: associate fragments of the same ball
            # by overlapping enclosing circles; preserve observed boundaries.
            seed_center, seed_radius = cv2.minEnclosingCircle(largest_contour)
            ball_contours = []
            for contour in contours:
                center, radius = cv2.minEnclosingCircle(contour)
                if np.hypot(center[0] - seed_center[0],
                            center[1] - seed_center[1]) <= seed_radius + radius:
                    ball_contours.append(contour)
            (cx, cy), _ = cv2.minEnclosingCircle(np.concatenate(ball_contours))
            pos = (cx, cy)  # Keep floating-point circle center for ROS output.

            cv2.drawContours(img, ball_contours, -1, (0, 255, 0), 2)
            cv2.drawMarker(img, (int(round(cx)), int(round(cy))), (255, 0, 0), cv2.MARKER_CROSS, 15, 2)

            text = f"Pos: ({cx:.2f}, {cy:.2f})"
            text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
            cv2.putText(img, text, ((width - text_size[0]) // 2, height - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

    return img, pos

# === [ Main Function ] ===
def main(args=None):
    rclpy.init(args=args)
    ros_node = VisionPublisher()
    
    if SHOW_IMAGE_WINDOW:
        # Window always follows the camera image's native resolution.
        cv2.namedWindow('UAV Downward Camera', cv2.WINDOW_AUTOSIZE)
    
    cam = GzCamRaw(CAMERA_TOPIC)
    
    fps_start_time = time.time()
    fps_frame_count = 0
    current_fps = 0.0

    try:
        while rclpy.ok():
            img, stamp = cam.get_next_image(timeout=0.01)
            current_time = time.time()
            
            if img is not None:
                processed_img, pos = process_and_find_red(img)
                
                if pos is not None and stamp is not None:
                    msg = PointStamped()
                    msg.header.stamp = Time(sec=stamp[0], nanosec=stamp[1])
                    msg.header.frame_id = "camera_link"
                    msg.point.x = float(pos[0])
                    msg.point.y = float(pos[1])
                    msg.point.z = 0.0
                    ros_node.pub.publish(msg)
                
                fps_frame_count += 1
                if (current_time - fps_start_time) >= FPS_UPDATE_PERIOD:
                    current_fps = fps_frame_count / (current_time - fps_start_time)
                    fps_start_time, fps_frame_count = current_time, 0
                    
                cv2.putText(processed_img, f"FPS: {current_fps:.1f}", (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 1, cv2.LINE_AA)
                
                if SHOW_IMAGE_WINDOW:
                    cv2.imshow('UAV Downward Camera', processed_img)

            if SHOW_IMAGE_WINDOW:
                cv2.waitKey(1)
            
            rclpy.spin_once(ros_node, timeout_sec=0)
                    
    except KeyboardInterrupt:
        pass
    finally:
        cam.cleanup() 
        ros_node.destroy_node()
        rclpy.try_shutdown()
        sys.exit(0)

if __name__ == "__main__":
    main()