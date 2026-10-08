#!/usr/bin/env python3
"""Compatibility-friendly entry point for the unified UAV_slung project."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    launch_file = os.path.join(
        get_package_share_directory("slung_ctrl"),
        "launch",
        "slung_gazebo.launch.py",
    )
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_file))
    ])
