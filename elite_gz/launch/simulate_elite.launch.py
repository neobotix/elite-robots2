# Neobotix GmbH
# Author: Pradheep Padmanabhan
# Contributor: Adarsh Karan K P

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_context import LaunchContext

import os
import xacro
"""
This code is used for simulating the robotic arm in an empty Gazebo world. 
"""

# OpaqueFunction is used to perform setup actions during launch through a Python function
def execution_stage(context: LaunchContext, use_arm_type_arg):

    use_arm_type = str(use_arm_type_arg.perform(context))

    robot_description_urdf = os.path.join(get_package_share_directory('elite_description'), 'urdf', 'elite_description.urdf.xacro')
    default_world_path = os.path.join(get_package_share_directory('elite_gz'), 'world', 'empty_world.world')
    bridge_config_file = os.path.join(get_package_share_directory('elite_gz'), 'config/gz_bridge', 'gz_bridge_config.yaml')

    # use_gz is set to True since this code launches the robot in simulation
    xacro_args = {'use_gz': "true", 'arm_type': use_arm_type}
    # Use xacro to process the file
    robot_description_xacro = xacro.process_file(robot_description_urdf, mappings=xacro_args).toxml()

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_model',
        output='screen',
        arguments=[
            '-topic', "robot_description",
            '-name', "elite"])
    
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        )
        , launch_arguments={'gz_args': ['-r ', default_world_path]}.items()
      )
    
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,  # Pass use_sim_time as True for simulation
            'robot_description':robot_description_xacro
            }]
        )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config_file}])

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
    )

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.dirname(get_package_share_directory('elite_description'))
    )

    launch_actions = [set_env_vars_resources, 
                      start_robot_state_publisher_cmd, 
                      gz_sim, 
                      gz_bridge, 
                      spawn_robot,
                      joint_state_broadcaster_spawner,
                      initial_joint_controller_spawner_started
                      ]

    return launch_actions

def generate_launch_description():
    # Declare launch arguments with default values and descriptions
    declare_use_arm_type_arg = DeclareLaunchArgument(
        'arm_type', default_value='ec66',
        description='Type of the arm to be launched(Currently only ec66 is supported)'
        )

    opq_func = OpaqueFunction(function = execution_stage,
                              args = [LaunchConfiguration('arm_type')]
                              )

    ld = LaunchDescription([
        declare_use_arm_type_arg,
        opq_func
    ])

    return ld

