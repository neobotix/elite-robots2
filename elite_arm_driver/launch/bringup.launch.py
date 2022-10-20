# Neobotix GmbH
# Author: Pradheep Padmanabhan
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():

    config = os.path.join(get_package_share_directory('elite_arm_driver'),'config','driver_params.yaml')

    neo_arm_driver =  Node(
        package='elite_arm_driver',
        executable="elite",
        name='elite_arm_driver',
        output='screen',
        parameters=[config],
        )

    return LaunchDescription([neo_arm_driver])