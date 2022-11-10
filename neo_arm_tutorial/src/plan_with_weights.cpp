// Neobotix GmbH

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("plan_with_weights");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("plan_with_weights", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher =
      node->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);
  while (planning_scene_diff_publisher->get_subscription_count() < 1)
  {
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }

  // Adding an object that the robot needs to hold
  moveit_msgs::msg::AttachedCollisionObject attached_object;
  attached_object.link_name = "flan";
  /* The header must contain a valid TF frame*/
  attached_object.object.header.frame_id = "flan";
  /* The id of the object */
  attached_object.object.id = "cylinder";

  /* A default pose */
  geometry_msgs::msg::Pose pose;
  pose.position.z = 0.03;
  pose.orientation.w = 1.0;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;
  primitive.dimensions.resize(2);
  primitive.dimensions[0] = 0.042;
  primitive.dimensions[1] = 0.08;

  attached_object.object.primitives.push_back(primitive);
  attached_object.object.primitive_poses.push_back(pose);

  attached_object.object.operation = attached_object.object.ADD;

  attached_object.touch_links = std::vector<std::string>{"flan"};

  // Adding an object that the robot needs to hold
  moveit_msgs::msg::PlanningScene planning_scene;
  planning_scene.world.collision_objects.push_back(attached_object.object);
  planning_scene.is_diff = true;
  planning_scene_diff_publisher->publish(planning_scene);

  RCLCPP_INFO(LOGGER, "Attaching the object to the hand and the world.");
  planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
  planning_scene.robot_state.is_diff = true;
  planning_scene_diff_publisher->publish(planning_scene);

  // ToDo detach the objects, if needed.

  rclcpp::shutdown();
  return 0;
}
