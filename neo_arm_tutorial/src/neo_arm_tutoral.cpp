#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <visualization_msgs/msg/marker_array.hpp>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <chrono>

#include <math.h>

using namespace std::chrono_literals;

// #include <moveit_visual_tools/moveit_visual_tools.h>

 visualization_msgs::msg::MarkerArray publish_markers(std::vector<geometry_msgs::msg::Pose> Poses) {
  visualization_msgs::msg::MarkerArray marker_array;

  for (size_t i = 0; i < Poses.size(); i++) {
    // Draw a green arrow at the waypoint pose
    visualization_msgs::msg::Marker arrow_marker;
    arrow_marker.header.frame_id = "base_link";
    arrow_marker.id = (int)i;
    arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
    arrow_marker.action = visualization_msgs::msg::Marker::ADD;
    arrow_marker.pose = Poses[i];
    arrow_marker.scale.x = 0.3;
    arrow_marker.scale.y = 0.05;
    arrow_marker.scale.z = 0.02;
    arrow_marker.color.r = 0;
    arrow_marker.color.g = 255;
    arrow_marker.color.b = 0;
    arrow_marker.color.a = 1.0f;
    arrow_marker.lifetime = rclcpp::Duration(0s);
    arrow_marker.frame_locked = false;
    marker_array.markers.push_back(arrow_marker);

    // Draw a red circle at the waypoint pose
    visualization_msgs::msg::Marker circle_marker;
    circle_marker.id = (int)i;
    circle_marker.header.frame_id = "base_link";
    circle_marker.type = visualization_msgs::msg::Marker::SPHERE;
    circle_marker.action = visualization_msgs::msg::Marker::ADD;
    circle_marker.pose = Poses[i];
    circle_marker.scale.x = 0.05;
    circle_marker.scale.y = 0.05;
    circle_marker.scale.z = 0.05;
    circle_marker.color.r = 255;
    circle_marker.color.g = 0;
    circle_marker.color.b = 0;
    circle_marker.color.a = 1.0f;
    circle_marker.lifetime = rclcpp::Duration(0s);
    circle_marker.frame_locked = false;
    marker_array.markers.push_back(circle_marker);

    // // Draw the waypoint number
    // visualization_msgs::msg::Marker marker_text;
    // marker_text.header = Poses[i].header;
    // marker_text.id = getUniqueId();
    // marker_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    // marker_text.action = visualization_msgs::msg::Marker::ADD;
    // marker_text.pose = Poses[i].pose;
    // marker_text.pose.position.z += 0.2;  // draw it on top of the waypoint
    // marker_text.scale.x = 0.07;
    // marker_text.scale.y = 0.07;
    // marker_text.scale.z = 0.07;
    // marker_text.color.r = 0;
    // marker_text.color.g = 255;
    // marker_text.color.b = 0;
    // marker_text.color.a = 1.0f;
    // marker_text.lifetime = rclcpp::Duration(0s);
    // marker_text.frame_locked = false;
    // marker_text.text = "wp_" + std::to_string(i + 1);
    // marker_array->markers.push_back(marker_text);
  }

return marker_array;
}

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "neo_arm_tutorial",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  auto array_publisher =
    node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "waypoints",
    rclcpp::QoS(1).transient_local());

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("neo_arm_tutorial");

  static const std::string PLANNING_GROUP = "arm";

  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  geometry_msgs::msg::PoseStamped current_pose;

  current_pose = move_group.getCurrentPose();

  moveit::core::RobotState start_state(*move_group.getCurrentState());
  geometry_msgs::msg::Pose start_pose2;
  start_pose2 = current_pose.pose;
  start_state.setFromIK(joint_model_group, start_pose2);
  move_group.setStartState(start_state);

  // move_group.setPoseReferenceFrame("flan");

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(start_pose2);

  geometry_msgs::msg::Pose target_pose3 = start_pose2;
  // target_pose3.position.z = 0.5;

  for(int i = 0; i<=500; i++) {
    target_pose3.position.x = sin(2*3.14*i*0.01) * 0.1 + 0.5;
    target_pose3.position.y = (i*0.01)* 0.1 - 0.05;
    waypoints.push_back(target_pose3);
  }

  // target_pose3.position.x -= 0.2;
  // waypoints.push_back(target_pose3);  // down

  // target_pose3.position.x -= 0.2;
  // waypoints.push_back(target_pose3);  // right

  // target_pose3.position.x -= 0.2;
  // waypoints.push_back(target_pose3);  // up and left

  array_publisher->publish(publish_markers(waypoints));

  // We want the Cartesian path to be interpolated at a resolution of 1 cm
  // which is why we will specify 0.01 as the max step in Cartesian
  // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
  // Warning - disabling the jump threshold while operating real hardware can cause
  // large unpredictable motions of redundant joints and could be a safety issue
  moveit_msgs::msg::RobotTrajectory trajectory;
  moveit_msgs::msg::RobotTrajectory trajectory_slow; //optimizing the trajectory

  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  RCLCPP_INFO(logger, "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

  trajectory_processing::IterativeParabolicTimeParameterization iptp(100, 0.05);
  robot_trajectory::RobotTrajectory r_trajec(move_group.getRobotModel(), PLANNING_GROUP);
  r_trajec.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);
  iptp.computeTimeStamps(r_trajec, 0.03, 0.03);
  r_trajec.getRobotTrajectoryMsg(trajectory_slow);

  move_group.execute(trajectory_slow);

  // Next step goes here

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}