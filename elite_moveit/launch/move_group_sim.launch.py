# Author: Pradheep Padmanabhan
# Contributor: Adarsh Karan K P

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ParameterFile
from moveit_configs_utils import MoveItConfigsBuilder
from pathlib import Path
    
def launch_setup(context, *args, **kwargs):

    # Initialize Arguments
    arm_type = LaunchConfiguration("arm_type")
    use_gz = LaunchConfiguration("use_gz")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")

    urdf = os.path.join(get_package_share_directory('elite_moveit'),
        'config',
        'ec66_simulation.urdf.xacro')

    # MoveIt Configuration
    srdf = os.path.join(get_package_share_directory('elite_moveit'),
        'config',
        'ec66.srdf')
    
    moveit_config = (
        MoveItConfigsBuilder("elite", package_name="elite_moveit")
        .robot_description_semantic(file_path=srdf)
        .robot_description(file_path=urdf, mappings={"arm_type": arm_type, "use_gz": use_gz})
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
        ],
    )

    # rviz with moveit configuration
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "moveit.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {
                "use_sim_time": use_sim_time,
            },
        ],
    )

    nodes_to_start = [move_group_node, rviz_node]

    return nodes_to_start

def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'arm_type', 
            default_value='ec66',
            choices=['ec66'],
            description='Arm Types:\n'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="elite_moveit",
            description='MoveIt config package with robot SRDF/XACRO files. Usually the argument\n'
            '\t is not set, it enables use of a custom moveit config.',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description='Make MoveIt to use simulation time.\n'
              '\t This is needed for the trajectory planing in simulation.',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_gz",
            default_value="true",
            description="Whether to enable Gazebo simulation.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz", 
            default_value="true", 
            description="Launch RViz?"
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

