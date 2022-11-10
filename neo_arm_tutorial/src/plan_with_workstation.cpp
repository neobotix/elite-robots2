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

static const rclcpp::Logger LOGGER = rclcpp::get_logger("plan_with_workstation");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("plan_with_workstation", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher =
      node->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);
  while (planning_scene_diff_publisher->get_subscription_count() < 1)
  {
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }

  // Adding an object that the environment needs to hold
  moveit_msgs::msg::AttachedCollisionObject env_object;
  env_object.link_name = "base_link";
  /* The header must contain a valid TF frame*/
  env_object.object.header.frame_id = "base_link";
  /* The id of the object */
  env_object.object.id = "BOX";

  /* A default pose for floor */
  geometry_msgs::msg::Pose pose_floor;
  pose_floor.position.z = 0.0;
  pose_floor.orientation.w = 1.0;

  shape_msgs::msg::SolidPrimitive rec;
  rec.type = rec.BOX;
  rec.dimensions.resize(3);
  rec.dimensions[0] = 2.5;
  rec.dimensions[1] = 2.5;
  rec.dimensions[2] = 0.0;

  env_object.object.primitives.push_back(rec);
  env_object.object.primitive_poses.push_back(pose_floor);

  env_object.object.operation = env_object.object.ADD;

  env_object.touch_links = std::vector<std::string>{"base_link"};

  moveit_msgs::msg::PlanningScene planning_scene;
  planning_scene.world.collision_objects.push_back(env_object.object);
  planning_scene.is_diff = true;
  planning_scene_diff_publisher->publish(planning_scene);

  RCLCPP_INFO(LOGGER, "Attaching the object to the environment");

  // ToDo detach the objects, if needed.

  rclcpp::shutdown();
  return 0;
}
