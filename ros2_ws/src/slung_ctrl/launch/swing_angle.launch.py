# ==============================================================================
# @File  : swing_angle.launch.py
# @Brief : Core swing-estimation stack used by ADRC experiments.
#
# Starts only the runtime chain needed for the current method:
#   Gazebo truth bridge -> truth estimator
#   raw vision -> timestamp aligner -> ESO fusion
#
# Optional comparison estimators (CV compensation / standard ESO / EKF) are
# moved to swing_angle_baselines.launch.py so the normal runtime graph stays
# compact.
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

    truth_node = Node(
        package='slung_ctrl',
        executable='swing_angle_truth',
        output='screen'
    )

    cv_node = Node(
        package='slung_ctrl',
        executable='swing_angle_cv',
        output='screen'
    )

    time_aligner_node = Node(
        package='slung_ctrl',
        executable='vision_time_aligner.py',
        output='screen'
    )

    fusion_node = Node(
        package='slung_ctrl',
        executable='swing_angle_eso_fusion',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        bridge_node,
        truth_node,
        cv_node,
        time_aligner_node,
        fusion_node,
    ])
