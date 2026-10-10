# ==============================================================================
# @File  : swing_angle_baselines.launch.py
# @Brief : Optional comparison estimators for offline/paper experiments.
#
# Run this alongside swing_angle.launch.py only when CV-comp / standard ESO / EKF
# comparison topics are needed. These nodes are intentionally not part of the
# default runtime chain.
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

    return LaunchDescription([
        Node(
            package='slung_ctrl',
            executable='swing_angle_cv_comp',
            output='screen'
        ),
        Node(
            package='slung_ctrl',
            executable='swing_angle_eso',
            output='screen',
            parameters=[config_file]
        ),
        Node(
            package='slung_ctrl',
            executable='swing_angle_ekf',
            output='screen'
        ),
    ])
