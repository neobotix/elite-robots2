# Neobotix GmbH
# Author: Pradheep Padmanabhan

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition
import os
from pathlib import Path
from launch.launch_context import LaunchContext

def execution_stage(context: LaunchContext):
    ip_address = LaunchConfiguration('ip_address')
    auto_connect = LaunchConfiguration('auto_connect')
    use_fake = LaunchConfiguration('use_fake')
    timer_period = LaunchConfiguration('timer_period')
    start_rviz = LaunchConfiguration('start_rviz')

    elite_description = os.path.join(get_package_share_directory('elite_description'), 'launch')

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
            PythonLaunchDescriptionSource([elite_description, '/elite_description.launch.py']),
            condition=IfCondition(start_rviz)
        )

    urdf = os.path.join(get_package_share_directory('elite_description'), 'urdf', 'ec66_description_real.urdf')

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description':Command([
            "xacro", " ", urdf])}]
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

    launch_args = []
    launch_args.extend([ip_addr_launch_arg,
        auto_connect_launch_arg,
        use_fake_launch_arg,
        timer_period_launch_arg,
        start_rviz_launch_arg])

    opq_function = OpaqueFunction(function=execution_stage)
    return LaunchDescription(launch_args + [opq_function])
