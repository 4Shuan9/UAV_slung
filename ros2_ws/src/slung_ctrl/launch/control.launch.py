#!/usr/bin/env python3
"""Select one controller and an explicit estimator source."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def create_controller(context):
    controller = LaunchConfiguration('controller').perform(context)
    estimator = LaunchConfiguration('estimator').perform(context)
    sim = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    root = get_package_share_directory('slung_ctrl')
    prefix = '/uav/swing/' + estimator
    if controller == 'adrc':
        if estimator not in ['eso', 'eso_delay', 'ekf3']:
            raise RuntimeError('ADRC requires a synchronized state: choose eso, eso_delay or ekf3; use controller:=pd for ekf2')
        config = 'control_ekf.yaml' if estimator == 'ekf3' else 'control_eso.yaml'
        return [Node(package='slung_ctrl', executable='slung_control_adrc', output='screen', parameters=[
            os.path.join(root, 'config', config), {'use_sim_time': sim, 'state_topic': prefix + '/state'}])]
    return [Node(package='slung_ctrl', executable='slung_control_pd', output='screen',
                 parameters=[os.path.join(root, 'config', 'control_pd.yaml'), {'use_sim_time': sim}],
                 remappings=[('/uav/swing/eso_delay/angle', prefix + '/angle'),
                             ('/uav/swing/eso_delay/rate', prefix + '/rate')])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('controller', default_value='adrc', choices=['adrc', 'pd']),
        DeclareLaunchArgument('estimator', default_value='eso_delay', choices=['eso_delay', 'ekf3', 'eso', 'ekf2']),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=create_controller),
    ])
