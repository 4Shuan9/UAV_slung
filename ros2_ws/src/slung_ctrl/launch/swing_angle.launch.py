# ==============================================================================
# @File     : swing_angle.launch.py
# @Author   : 4Shuan9
# @Date     : 2026-07-31
# @Brief    : 摆角估计算法集群与真值桥接一键启动脚本
# 
# @Main Work: 
#   * 数据桥接：启动 `ros_gz_bridge` 节点，将 Gazebo 物理引擎中无人机与吊挂物的精确 Pose 数组桥接至 ROS 2 环境中，为真值计算提供基础
#   * 对比基准：启动 `swing_angle_truth` 节点，解算绝对零误差、零延迟的系统物理摆角真值
#   * 算法并行：一键并行启动 `swing_angle_cv`（纯视觉观测）、`swing_angle_cv_comp`（纯运动学补偿）、`swing_angle_ekf`（扩展卡尔曼滤波）、`swing_angle_eso`（标准扩张状态观测器）以及 `swing_angle_eso_fusion`（高级融合观测器）共 5 大算法节点
#   * 参数共享：为关联算法动态加载 `eso_params.yaml`，统一分配物理参量（如重力常数、绳长、观测器带宽等）
# ==============================================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('slung_ctrl'),
        'config',
        'eso_params.yaml'
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/payload_ball/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose',
            '/model/x500_mono_cam_down/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose'
        ],
        output='screen'
    )

    swing_angle_truth_node = Node(
        package='slung_ctrl',
        executable='swing_angle_truth',
        output='screen'
    )

    swing_angle_cv_node = Node(
        package='slung_ctrl',
        executable='swing_angle_cv',
        output='screen'
    )

    swing_angle_cv_comp_node = Node(
        package='slung_ctrl',
        executable='swing_angle_cv_comp',
        output='screen'
    )

    swing_angle_eso_node = Node(
        package='slung_ctrl',
        executable='swing_angle_eso',
        output='screen',
        parameters=[config_file]
    )

    swing_angle_eso_fusion_node = Node(
        package='slung_ctrl',
        executable='swing_angle_eso_fusion',
        output='screen',
        parameters=[config_file]
    )

    swing_angle_ekf_node = Node(
        package='slung_ctrl',
        executable='swing_angle_ekf',
        output='screen'
    )

    return LaunchDescription([
        bridge_node,
        swing_angle_truth_node,
        swing_angle_cv_node,
        swing_angle_cv_comp_node,
        swing_angle_eso_node,
        swing_angle_eso_fusion_node,
        swing_angle_ekf_node
    ])