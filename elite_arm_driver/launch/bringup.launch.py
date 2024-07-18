# Neobotix GmbH
# Author: Pradheep Padmanabhan

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, Command
from launch_ros.actions import Node
from launch.conditions import IfCondition
import os
from pathlib import Path
import xacro
from launch.launch_context import LaunchContext

def execution_stage(context: LaunchContext):
    ip_address = LaunchConfiguration('ip_address')
    auto_connect = LaunchConfiguration('auto_connect')
    use_fake = LaunchConfiguration('use_fake')
    timer_period = LaunchConfiguration('timer_period')
    start_rviz = LaunchConfiguration('start_rviz')
    use_arm_type = LaunchConfiguration('arm_type')

    elite_description_pkg = get_package_share_directory('elite_description')

    bringup_arm_driver = Node(
        package="elite_arm_driver",
        executable="elite",
        parameters=[{
            'ip_address': ip_address,
            'auto_connect': auto_connect,
            'use_fake': use_fake,
            'timer_period': timer_period}]
    )

    rviz_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(elite_description_pkg, 'launch', 'elite_description.launch.py')),
            condition=IfCondition(start_rviz)
    )

    robot_description_urdf = os.path.join(elite_description_pkg, 'urdf', 'elite_description.urdf.xacro')
    # use_gazebo is set to False since no simulation is involved
    xacro_args = {'use_gazebo': "false", 'arm': use_arm_type}
    # Use xacro to process the file
    robot_description_xacro = xacro.process_file(robot_description_urdf, mappings=xacro_args).toxml()

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description':Command([
            "xacro", " ", robot_description_xacro])}]
        )

    return [
        bringup_arm_driver,
        rviz_launch,
        start_robot_state_publisher_cmd]

def generate_launch_description():
    ip_addr_launch_arg = DeclareLaunchArgument(
        'ip_address',
        default_value='10.1.30.80')

    auto_connect_launch_arg = DeclareLaunchArgument(
        'auto_connect',
        default_value='True')

    use_fake_launch_arg = DeclareLaunchArgument(
        'use_fake',
        default_value='False')

    timer_period_launch_arg = DeclareLaunchArgument(
        'timer_period',
        default_value=TextSubstitution(text='0.01'))

    start_rviz_launch_arg = DeclareLaunchArgument(
        'start_rviz',
        default_value='False')

    use_arm_type_arg = DeclareLaunchArgument(
        'arm_type',
        default_value='ec66')

    launch_args = []
    launch_args.extend([ip_addr_launch_arg,
        auto_connect_launch_arg,
        use_fake_launch_arg,
        timer_period_launch_arg,
        start_rviz_launch_arg,
        use_arm_type_arg])

    opq_function = OpaqueFunction(function=execution_stage)
    return LaunchDescription(launch_args + [opq_function])
