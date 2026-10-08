#!/usr/bin/env python3

# 状态：已弃用
# ==============================================================================
# @File     : swing_angle.py
# @Author   : 4Shuan9
# @Date     : 2026-07-31
# @Brief    : 摆角真值计算脚本 Python 版
# 
# @Main Work: 
#   * 原始功能：通过 message_filters 同步订阅机体与吊挂物位姿，通过坐标转换计算真实摆角，并发布至笛卡尔与极坐标系话题
#   * 冗余说明：该文件为真值计算的早期 Python 验证版本，当前已被 C++ 节点 `swing_angle_truth.cpp` 替代
#             在 200Hz 的高频仿真数据流下，C++ 版本具备更高的执行效率与更严谨的时间戳同步性能
#             当前的 Launch 启动脚本 (`run_estimators.launch.py`) 也已切换为调用 C++ 节点
#   * 处理建议：可作为四元数/旋转矩阵坐标系转换算法的易读版原型保留，或直接删除
# ==============================================================================

# === [ Includes & Configuration ] ===
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Vector3Stamped
import message_filters
import numpy as np
import math

class SwingAngleCalculator(Node):
    def __init__(self):
        super().__init__('swing_angle_calculator')
        
        # 1. 订阅者：使用 message_filters 订阅带有时间戳的 PoseStamped
        self.sub_uav = message_filters.Subscriber(
            self, PoseStamped, '/model/x500_mono_cam_down/pose')
        self.sub_payload = message_filters.Subscriber(
            self, PoseStamped, '/model/payload_ball/pose')
            
        # 时间戳近似同步器：队列大小10，容忍度0.01秒 (200Hz的周期是0.005s，容忍度设为2个周期)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_uav, self.sub_payload], queue_size=10, slop=0.01)
        self.ts.registerCallback(self.sync_callback)
            
        # 2. 发布者：使用新设计的真值(truth)命名空间，并升级为带有时间戳的 Vector3Stamped
        self.pub_cartesian = self.create_publisher(
            Vector3Stamped, '/uav/truth/swing_angle/cartesian', 10)
        self.pub_polar = self.create_publisher(
            Vector3Stamped, '/uav/truth/swing_angle/polar', 10)

    def sync_callback(self, uav_msg, payload_msg):
        """只有当两个话题的时间戳对齐时，才会触发此回调，极大保证了高频计算的准确性"""
        
        # --- 1. 提取位置向量并计算绝对绳长 ---
        Pu = np.array([uav_msg.pose.position.x, 
                       uav_msg.pose.position.y, 
                       uav_msg.pose.position.z])
                       
        Pl = np.array([payload_msg.pose.position.x, 
                       payload_msg.pose.position.y, 
                       payload_msg.pose.position.z])
                       
        r_world = Pl - Pu
        rope_length = np.linalg.norm(r_world)
        
        # 保护：防止刚初始化时重合导致除以零
        if rope_length < 1e-5:
            return

        # --- 2. 提取四元数，进行归一化保护 ---
        q = uav_msg.pose.orientation
        norm = math.sqrt(q.w**2 + q.x**2 + q.y**2 + q.z**2)
        if norm < 1e-5:
            return
        w, x, y, z = q.w/norm, q.x/norm, q.y/norm, q.z/norm
        
        # 将四元数转换为旋转矩阵 (直接内联计算，提升200Hz下的运行效率)
        R_u = np.array([
            [1 - 2*(y**2 + z**2), 2*(x*y - w*z),     2*(x*z + w*y)],
            [2*(x*y + w*z),       1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
            [2*(x*z - w*y),       2*(y*z + w*x),       1 - 2*(x**2 + y**2)]
        ])
        
        # --- 3. 坐标系转换 (矩阵乘法) ---
        # 检查坐标系方向：Gazebo默认ENU，飞机下方为-Z。
        # R_u.T 将世界坐标系向量转至机体坐标系。
        r_body = R_u.T @ r_world  
        xb, yb, zb = r_body[0], r_body[1], r_body[2]
        
        # --- 4. 计算摆角 (笛卡尔 & 极坐标) ---
        
        # 方案一：笛卡尔坐标系投影角 (Theta X, Theta Y)
        theta_x_deg = math.degrees(math.atan2(xb, -zb))
        theta_y_deg = math.degrees(math.atan2(yb, -zb))
        
        # 方案二：极坐标系 (总夹角 Alpha, 方位角 Beta)
        cos_alpha = max(-1.0, min(1.0, -zb / rope_length))
        alpha_deg = math.degrees(math.acos(cos_alpha))
        beta_deg = math.degrees(math.atan2(yb, xb))
        
        # --- 5. 发布数据 (使用 Vector3Stamped 记录时间戳) ---
        
        # 笛卡尔坐标系数据
        msg_cart = Vector3Stamped()
        msg_cart.header.stamp = uav_msg.header.stamp  # 完美继承机体位姿的原始时间戳
        msg_cart.header.frame_id = 'uav_base_link'    # 指定参考坐标系
        msg_cart.vector.x = float(theta_x_deg)
        msg_cart.vector.y = float(theta_y_deg)
        msg_cart.vector.z = float(rope_length)
        self.pub_cartesian.publish(msg_cart)
        
        # 极坐标系数据
        msg_polar = Vector3Stamped()
        msg_polar.header.stamp = uav_msg.header.stamp # 完美继承机体位姿的原始时间戳
        msg_polar.header.frame_id = 'uav_base_link'   # 指定参考坐标系
        msg_polar.vector.x = float(alpha_deg)
        msg_polar.vector.y = float(beta_deg)
        msg_polar.vector.z = float(rope_length)
        self.pub_polar.publish(msg_polar)
        
        # --- 6. 规范化控制台输出 ---
        log_msg = (
            f"\n"
            f"  [真值-笛卡尔] 纵向(X): {theta_x_deg:>+7.2f}° | 横向(Y): {theta_y_deg:>+7.2f}°\n"
            f"  [真值-极坐标] 总角(α): {alpha_deg:>+7.2f}° | 方位(β): {beta_deg:>+7.2f}°\n"
            f"  [状态值] 实时绳长: {rope_length:>7.3f} m"
        )
        
        # 使用 ROS 2 的 throttle 机制，防止200Hz刷爆终端卡死
        self.get_logger().info(log_msg, throttle_duration_sec=0.05)

def main(args=None):
    rclpy.init(args=args)
    node = SwingAngleCalculator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()