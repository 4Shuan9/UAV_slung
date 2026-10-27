#!/usr/bin/env python3
"""One vision/truth chain, with one observer or all four observers."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def create_nodes(context):
    selected = LaunchConfiguration('estimator').perform(context)
    sim = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    vision = LaunchConfiguration('vision_topic').perform(context)
    mode = LaunchConfiguration('input_time_mode').perform(context)
    share = get_package_share_directory('slung_ctrl')
    clock = {'use_sim_time': sim}
    nodes = []
    if sim:
        nodes.append(Node(package='ros_gz_bridge', executable='parameter_bridge',
                          name='clock_bridge', arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], output='screen'))
    nodes += [
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='pose_bridge', arguments=[
            '/model/payload_ball/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose',
            '/model/x500_mono_cam_down/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose'], output='screen'),
        Node(package='slung_ctrl', executable='swing_truth', parameters=[clock], output='screen'),
        Node(package='slung_ctrl', executable='swing_vision', parameters=[clock], output='screen'),
    ]
    methods = ['eso', 'eso_delay', 'ekf2', 'ekf3'] if selected == 'all' else ([] if selected == 'none' else [selected])
    for method in methods:
        params = [os.path.join(share, 'config', method + '.yaml'), clock]
        if method in ['eso_delay', 'ekf3']:
            params.append({'input_time_mode': mode})
        nodes.append(Node(package='slung_ctrl', executable='swing_' + method, parameters=params,
                          remappings=[('/uav/swing/vision/angle', vision)], output='screen'))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('estimator', default_value='eso_delay',
                              choices=['eso_delay', 'ekf3', 'eso', 'ekf2', 'all', 'none'],
                              description='all starts eso, eso_delay, ekf2 and ekf3 once each'),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('vision_topic', default_value='/uav/swing/vision/angle'),
        DeclareLaunchArgument('input_time_mode', default_value='synchronized', choices=['synchronized', 'offset']),
        OpaqueFunction(function=create_nodes),
    ])
