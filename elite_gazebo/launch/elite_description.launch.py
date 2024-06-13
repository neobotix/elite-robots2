# Neobotix GmbH
# Author: Pradheep Padmanabhan

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration, Command
from launch_ros.actions import Node
import os
from pathlib import Path


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')

    urdf = os.path.join(get_package_share_directory('elite_description'), 'urdf', 'ec66_description_real.urdf')

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description':Command([
            "xacro", " ", urdf]), 'use_sim_time': use_sim_time}]
        )

    return LaunchDescription([start_robot_state_publisher_cmd])