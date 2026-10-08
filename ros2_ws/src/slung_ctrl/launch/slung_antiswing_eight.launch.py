# ==============================================================================
# @File     : slung_antiswing_eight.launch.py
# @Author   : 4Shuan9
# @Date     : 2026-07-31
# @Brief    : 8字轨迹（伯努利双纽线）生成节点启动脚本
# 
# @Main Work: 
#   * 节点启动：启动 `slung_ctrl` 包下的 `slung_antiswing_eight` 动态轨迹生成节点
#   * 参数加载：动态加载配置文件 `eight_params.yaml`，导入控制伯努利双纽线的缩放系数（轨迹范围）和角频率（运动速度）
# ==============================================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('slung_ctrl'), 'config', 'eight_params.yaml'
    )
    return LaunchDescription([
        Node(
            package='slung_ctrl',
            executable='slung_antiswing_eight', 
            name='slung_antiswing_eight',
            output='screen',
            parameters=[config_file]
        )
    ])