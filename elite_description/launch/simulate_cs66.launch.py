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
    default_world_path = os.path.join(get_package_share_directory('neo_gz_worlds'), 'worlds', 'neo_workshop.sdf')

    use_sim_time = LaunchConfiguration('use_sim_time', default='False')

    urdf = os.path.join(get_package_share_directory('elite_description'), 'urdf', 'cs66_description.urdf')
    doc = xacro.parse(open(urdf)) 
    xacro.process_doc(doc) 

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_model',
        output='screen',
        arguments=[
            '-topic', "robot_description",
            '-name', "ec66"])

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description':doc.toxml(), 'use_sim_time': use_sim_time}]
        )

    ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        )
        , launch_arguments={'ign_args': ['-r ', default_world_path]}.items()
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

    return LaunchDescription([start_robot_state_publisher_cmd, joint_state_broadcaster_spawner, initial_joint_controller_spawner_stopped, ignition, spawn_entity])