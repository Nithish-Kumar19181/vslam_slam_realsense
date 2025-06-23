#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command
import os

def generate_launch_description():
    description_pkg = FindPackageShare('hexapod_description').find('hexapod_description')
    urdf_xacro = os.path.join(description_pkg, 'urdf', 'hexapod.urdf.xacro')
    robot_description_content = Command(['xacro ', urdf_xacro])
    robot_description = {'robot_description': robot_description_content}

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {'use_sim_time': False}],
        ),

    ])