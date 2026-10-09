# ==============================================================================
# @File     : slung_antiswing.launch.py
# @Author   : 4Shuan9
# @Date     : 2026-07-31
# @Brief    : 核心防摆控制节点启动脚本
# 
# @Main Work: 
#   * 节点启动：启动 `slung_ctrl` 包下的 `slung_antiswing` 核心控制节点
#   * 参数加载：动态加载外部配置文件 `antiswing_params.yaml`，用于配置起飞、位置环、ADRC 参数，以及 disturbance 预限幅/低通/最终安全限幅参数
#   * 环境配置：标准输出定向至终端屏幕，方便开发人员实时监控无人机的状态机切换（起飞、防摆控制段）日志
# ==============================================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('slung_ctrl'), 'config', 'antiswing_params.yaml'
    )
    return LaunchDescription([
        Node(
            package='slung_ctrl',
            executable='slung_antiswing_ADRC', 
            name='slung_antiswing',
            output='screen',
            parameters=[config_file]
        )
    ])