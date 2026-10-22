#!/usr/bin/env python3
"""One vision/truth chain, with a selected observer and optional baselines."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def create_nodes(context):
    selected = LaunchConfiguration('estimator').perform(context)
    sim = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    baselines = LaunchConfiguration('baselines').perform(context).lower() == 'true'
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
    methods = [] if selected == 'none' else [selected]
    if baselines:
        methods += [m for m in ['eso', 'ekf2', 'vision_predictor'] if m not in methods]
    for method in methods:
        params = [clock]
        if method != 'vision_predictor':
            params.insert(0, os.path.join(share, 'config', method + '.yaml'))
        if method in ['eso_delay', 'ekf3']:
            params.append({'input_time_mode': mode})
        nodes.append(Node(package='slung_ctrl', executable='swing_' + method, parameters=params,
                          remappings=[('/uav/swing/vision/angle', vision)], output='screen'))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('estimator', default_value='eso_delay',
                              choices=['eso_delay', 'ekf3', 'eso', 'ekf2', 'vision_predictor', 'none']),
        DeclareLaunchArgument('baselines', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('vision_topic', default_value='/uav/swing/vision/angle'),
        DeclareLaunchArgument('input_time_mode', default_value='synchronized', choices=['synchronized', 'offset']),
        OpaqueFunction(function=create_nodes),
    ])
