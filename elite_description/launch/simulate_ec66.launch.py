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
import xacro


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')

    urdf = os.path.join(get_package_share_directory('elite_description'), 'urdf', 'ec66_description.urdf')
    doc = xacro.parse(open(urdf)) 
    xacro.process_doc(doc) 

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', "ec66", '-topic', "robot_description"],
        output='screen')

    default_world_path = os.path.join(get_package_share_directory('elite_description'), 'world', 'empty_world.world')

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description':doc.toxml(), 'use_sim_time': use_sim_time}]
        )

    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'world': default_world_path,
                'verbose': 'true',
            }.items(),
        )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
    )

    return LaunchDescription([start_robot_state_publisher_cmd, joint_state_broadcaster_spawner, initial_joint_controller_spawner_stopped, gazebo, spawn_entity])