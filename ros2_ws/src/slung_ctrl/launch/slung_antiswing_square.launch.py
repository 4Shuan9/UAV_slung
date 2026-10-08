# ==============================================================================
# @File     : slung_antiswing_square.launch.py
# @Author   : 4Shuan9
# @Date     : 2026-07-31
# @Brief    : 菱形/方形巡航轨迹生成节点启动脚本
# 
# @Main Work: 
#   * 节点启动：启动 `slung_ctrl` 包下的 `slung_antiswing_square` 轨迹生成节点
#   * 参数加载：动态加载配置文件 `square_params.yaml`，导入菱形轨迹的单边长度与无人机沿直线飞行时的巡航速度
# ==============================================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('slung_ctrl'), 'config', 'square_params.yaml'
    )
    return LaunchDescription([
        Node(
            package='slung_ctrl',
            executable='slung_antiswing_square', 
            name='slung_antiswing_square',
            output='screen',
            parameters=[config_file]
        )
    ])