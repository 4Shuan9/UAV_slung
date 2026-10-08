#!/usr/bin/env python3

# 状态：已弃用
# ==============================================================================
# @File     : gz_cam_raw.py
# @Author   : 4Shuan9
# @Date     : 2026-07-31
# @Brief    : Gazebo 原生图像接收测试脚本
# 
# @Main Work: 
#   * 原始功能：通过 gz-transport 单独订阅 Gazebo 底层相机图像流，并使用 OpenCV 窗口进行显示测试
#   * 冗余说明：该文件为早期的独立测试用脚本，其图像接收与解码逻辑已被完整集成到 `gz_cam_cv.py` 中
#             由于 `gz_cam_cv.py` 已具备更完善的特征识别与 ROS 2 消息发布功能，此文件不再参与实际的控制闭环
#   * 处理建议：可作为 gz-transport Python API 的基础调用参考存档，或直接从项目中删除以精简代码库
# ==============================================================================

# === [ Includes & Configuration ] ===
import time
import threading
import sys
import cv2
import numpy as np

from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node

CAMERA_TOPIC = "/world/UAV_slung_world/model/x500_mono_cam_down/link/camera_link/sensor/imager/image"
SHOW_IMAGE_WINDOW = True

# === [ Gazebo 原生图像接收与解码 ] ===
class GzCamRaw:
    def __init__(self, topic_name):
        self._node = Node()
        self._node.subscribe(Image, topic_name, self._cb)
        
        self._raw_msg = None 
        self._condition = threading.Condition()
        self.is_running = True

    # 1. 订阅回调 (轻量化处理)
    def _cb(self, msg: Image) -> None:
        if not self.is_running:
            return
            
        with self._condition:
            self._raw_msg = msg
            self._condition.notify_all()

    # 2. 主线程解码处理
    def get_next_image(self, timeout=None):
        with self._condition:
            if self._raw_msg is None:
                self._condition.wait_for(lambda: self._raw_msg is not None or not self.is_running, timeout=timeout)
            
            if self._raw_msg is None or not self.is_running:
                return None
                
            msg = self._raw_msg
            self._raw_msg = None 
            
        width = msg.width
        height = msg.height
        raw_data = msg.data
        
        if len(raw_data) % (width * height) != 0:
            print(f"[WARN] 图像数据大小异常: {len(raw_data)} bytes")
            return None
            
        channels = len(raw_data) // (width * height)
        
        try:
            np_image = np.frombuffer(raw_data, dtype=np.uint8).reshape((height, width, channels))
            
            if channels == 3:
                cv2_image = cv2.cvtColor(np_image, cv2.COLOR_RGB2BGR)
            elif channels == 4:
                cv2_image = cv2.cvtColor(np_image, cv2.COLOR_RGBA2BGR)
            elif channels == 1:
                cv2_image = np_image
            else:
                cv2_image = np_image 
                
            return cv2_image
            
        except Exception as e:
            print(f"[ERROR] 图像解码失败: {e}")
            return None

    # 3. 资源清理
    def cleanup(self):
        print("\n[INFO] 开始清理资源...")
        self.is_running = False
        with self._condition:
            self._condition.notify_all()
        if SHOW_IMAGE_WINDOW:
            cv2.destroyAllWindows()
        print("[INFO] 资源已安全释放。")

# === [ 主程序控制流 ] ===
def main():
    print(f"正在连接到话题: {CAMERA_TOPIC}")
    
    # 1. 窗口初始化
    if SHOW_IMAGE_WINDOW:
        cv2.namedWindow('UAV Downward Camera', cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        cv2.resizeWindow('UAV Downward Camera', 960, 720)
        print("图像显示已开启，按 'q' 键或 'Ctrl+C' 退出。")
    else:
        print("图像显示已关闭，程序将在后台持续获取图像数据。")
    
    cam = GzCamRaw(CAMERA_TOPIC)
    
    try:
        # 2. 主循环
        while True:
            img = cam.get_next_image(timeout=1.0)
            
            if img is not None:
                if SHOW_IMAGE_WINDOW:
                    cv2.imshow('UAV Downward Camera', img)
            
            if SHOW_IMAGE_WINDOW:
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("\n[INFO] 接收到退出指令。")
                    break
                    
    except KeyboardInterrupt:
        print("\n[INFO] 节点被手动中断 (KeyboardInterrupt)")
    except Exception as e:
        print(f"\n[FATAL] 未处理的异常: {str(e)}")
    finally:
        # 3. 资源清理
        cam.cleanup() 
        sys.exit(0)

if __name__ == "__main__":
    main()