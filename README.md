# elite-robots2

elite-robots2 is an open source and community-driven package consisting of the ros-drivers for the arms manufactured by Elite. This ROS 2 package is a direct migration of the ROS package, which can be found here https://github.com/Elite-Robots/ROS 

This package was developed by Neobotix. This package is currently maintained by Neobotix and Elite Robots. 

![new_neobotix_1920px](https://github.com/neobotix/elite-robots2/assets/20242192/1377441b-a9bb-42bd-9a8b-c190773d13d4)


At the moment, this ROS 2 package supports the CS66 variant. But a Pull Request would well and truly be accepted for other variants of the Elite arm. 

Also this package consists of the simulation for the CS 66 arm.

Please feel free to use the issue tracker for requesting any feature requests or bug clearance.

We have tested the ROS 2 node in **ROS-Humble** (Ubuntu 22.04). All the developments must be made in the **ROS-Rolling** distro (Ubuntu 22.04).  

## Installation

We assume that ROS-Rolling or ROS-Humble is installed and the user has a basic understanding of ROS 2. Also make sure the installed distribution is sourced.

1. Install moveit

`sudo apt-get install ros-$ROS_DISTRO-moveit-*`

2. Install Gazebo packages and ros2 controllers

`sudo apt-get install ros-$ROS_DISTRO-gazebo-ros ros-$ROS_DISTRO-gazebo-ros2-control ros-humble-ros2-controllers ros-humble-ros2-control`

## Package setup

For this step, we need to create a colcon_ws (more details on the build system: https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Documentation.html)

```
mkdir ~/ros_workspace/src
git clone git@github.com:neobotix/elite-robots2.git
colcon build --symlink-install
```

Do not forget to source the setup script of your workspace. For that you need to do:

```
cd ~/ros_workspace/
source install/setup.bash
```

If you do not want to do this step all the time, then do

`echo "source your_colcon_workspace/install/setup.bash" >> ~/.bashrc`

This add's your setup script to the bashrc (need to restart the terminal for the first time after you add it). 

## Launching the nodes

### Simulated robot:

1. Starting the necessary Gazebo-ROS nodes, ros2 joint controllers and joint state broadcasters

`ros2 launch elite_description simulate_cs66.launch.py`

2. Start the planning pipeline by launching:

`ros2 launch elite_moveit move_group_sim.launch.py use_sim_time:=True`

You would see the RViz with the robot model spawned. 

### Real robot:

Coming soon .....
