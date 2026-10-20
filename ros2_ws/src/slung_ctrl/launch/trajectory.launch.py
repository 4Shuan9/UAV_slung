#!/usr/bin/env python3
"""Publish a figure-eight, square, or externally supplied waypoint reference."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def create_trajectory(context):
    kind = LaunchConfiguration('type').perform(context)
    sim = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    params = [{'use_sim_time': sim}]
    if kind != 'waypoint':
        params.insert(0, os.path.join(get_package_share_directory('slung_ctrl'), 'config', kind + '.yaml'))
    return [Node(package='slung_ctrl', executable='trajectory_' + kind, parameters=params, output='screen')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('type', default_value='eight', choices=['eight', 'square', 'waypoint']),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=create_trajectory),
    ])
