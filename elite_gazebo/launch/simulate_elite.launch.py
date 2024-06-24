# Neobotix GmbH
# Author: Pradheep Padmanabhan

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import xacro
"""
This code is used for simulating the robotic arm in an empty Gazebo world. 
"""

# OpaqueFunction is used to perform setup actions during launch through a Python function
def launch_setup(context, use_sim_time_arg):
    # Create a list to hold all the nodes
    launch_actions = []
    use_sim_time = use_sim_time_arg.perform(context).lower() == 'true'

    # Get the required paths for the world and robot robot_description_urdf
    robot_description_urdf = os.path.join(get_package_share_directory('elite_description'), 'urdf', 'ec66_description.urdf.xacro')
    default_world_path = os.path.join(get_package_share_directory('elite_gazebo'), 'world', 'empty_world.world')
    # use_gazebo is set to True since this code launches the robot in simulation
    xacro_args = {'use_gazebo': "true"}
    # Use xacro to process the file
    robot_description_xacro = xacro.process_file(robot_description_urdf, mappings=xacro_args).toxml()

    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', "ec66",'-topic', '/robot_description'], 
        output='screen'
    )

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description':robot_description_xacro, 'use_sim_time': use_sim_time}]
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

    launch_actions.append(spawn_entity)
    launch_actions.append(gazebo)
    launch_actions.append(start_robot_state_publisher_cmd)
    launch_actions.append(joint_state_broadcaster_spawner)
    launch_actions.append(initial_joint_controller_spawner_stopped)

    return launch_actions

def generate_launch_description():
    ld = LaunchDescription()

    # Declare launch arguments with default values and descriptions
    declare_use_sim_time_arg = DeclareLaunchArgument(
            'use_sim_time', default_value='True',
            description='Use simulation clock if true (True/False)'
        )

    # Create launch configuration variables
    use_sim_time_arg = LaunchConfiguration('use_sim_time')

    ld.add_action(declare_use_sim_time_arg)

    context_arguments = [use_sim_time_arg]
    opq_func = OpaqueFunction(  
        function = launch_setup,
        args = context_arguments
        )

    ld.add_action(opq_func)

    return ld
