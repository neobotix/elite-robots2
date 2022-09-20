cmake_minimum_required(VERSION 3.5)
project(elite_msgs)

## By default build with code optimization and no debug info
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotState.msg"
   # -- robot --
   "srv/RobotServoOn.srv"
   # -- io --
   "srv/SetIO.srv"
   "srv/SetAnalogIO.srv"
   # -- move --
   "srv/StopMove.srv"
   "srv/JointMove.srv"
   "srv/CartMove.srv"
   "srv/JointTrajectoryMove.srv"
   # -- kinematic --
   "srv/InverseKinematic.srv"
   "srv/ForwardKinematic.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_package()