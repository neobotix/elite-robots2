# Neobotix GmbH
# Author: Pradheep Padmanabhan

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.actions import TimerAction
import os
from pathlib import Path
import xacro
"""
This code is used for debugging, quick testing, and visualization of the robotic arm in Rviz. 
"""

# OpaqueFunction is used to perform setup actions during launch through a Python function
def launch_setup(context, use_sim_time_arg, use_joint_state_publisher_gui_arg, use_arm_type_arg):
    # Create a list to hold all the nodes
    launch_actions = []

    elite_description_pkg = get_package_share_directory('elite_description')

    use_sim_time = use_sim_time_arg.perform(context).lower() == 'true'
    use_joint_state_publisher_gui = use_joint_state_publisher_gui_arg.perform(context)
    use_arm_type = use_arm_type_arg.perform(context)

    robot_description_urdf = os.path.join(elite_description_pkg, 'urdf', 'elite_description.urdf.xacro')

    # use_gazebo is set to False since no simulation is involved
    xacro_args = {'use_gazebo': "false", 'arm': use_arm_type}
    # Use xacro to process the file
    robot_description_xacro = xacro.process_file(robot_description_urdf, mappings=xacro_args).toxml()

    rviz_config = os.path.join(elite_description_pkg, 'rviz', 'elite_rviz.rviz')

    # Start the joint state publisher gui only if use_joint_state_publisher_gui is True
    start_joint_state_publisher_gui_cmd = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(use_joint_state_publisher_gui),
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=IfCondition(PythonExpression(['not ', use_joint_state_publisher_gui])),
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description':robot_description_xacro, 'use_sim_time': use_sim_time}],
    )

    # Rviz node
    start_rviz_cmd = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d' + rviz_config]
    )

    launch_actions.append(start_rviz_cmd)
    launch_actions.append(start_robot_state_publisher_cmd)
    launch_actions.append(start_joint_state_publisher_cmd)
    launch_actions.append(start_joint_state_publisher_gui_cmd)

    return launch_actions

def generate_launch_description():
    ld = LaunchDescription()

    # Declare launch arguments with default values and descriptions
    declare_use_sim_time_arg = DeclareLaunchArgument(
            'use_sim_time', default_value='False',
            description='Use simulation clock if True (True/False)'
        )
    declare_use_joint_state_publisher_gui_arg = DeclareLaunchArgument(
            'use_joint_state_publisher_gui', default_value='True',
            description='Use joint state publisher gui if True (True/False)'
        )
    declare_use_arm_type_arg = DeclareLaunchArgument(
            'arm_type', default_value='ec66',
            description='Type of arm to be launched (ec66)'
        )

    use_sim_time_arg = LaunchConfiguration('use_sim_time')
    use_joint_state_publisher_gui_arg = LaunchConfiguration('use_joint_state_publisher_gui')
    use_arm_type_arg = LaunchConfiguration('arm_type')

    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_use_joint_state_publisher_gui_arg)
    ld.add_action(declare_use_arm_type_arg)

    context_arguments = [use_sim_time_arg, use_joint_state_publisher_gui_arg, use_arm_type_arg]
    opq_func = OpaqueFunction(  
        function = launch_setup,
        args = context_arguments
        )

    ld.add_action(opq_func)

    return ld
