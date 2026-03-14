#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometry_msgs/msg/pose_stamped.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pose_goal_demo", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // MoveGroupInterface needs a spinner
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  std::thread spinner([&exec]() { exec.spin(); });

  // IMPORTANT: planning group name must match the UR MoveIt config.
  // Common names: "ur_manipulator" or "manipulator"
  const std::string planning_group = node->declare_parameter<std::string>("planning_group", "ur_manipulator");

  moveit::planning_interface::MoveGroupInterface move_group(node, planning_group);

  move_group.setPlanningTime(5.0);
  move_group.setNumPlanningAttempts(5);
  move_group.setMaxVelocityScalingFactor(0.2);
  move_group.setMaxAccelerationScalingFactor(0.2);

  // ee link: "tool0" or "ee_link"
  RCLCPP_INFO(node->get_logger(), "Planning frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(node->get_logger(), "End effector link: %s", move_group.getEndEffectorLink().c_str());

  geometry_msgs::msg::PoseStamped target;
  target.header.frame_id = move_group.getPlanningFrame();  // usually base_link
  target.pose.position.x = 0.45;
  target.pose.position.y = 0.2;
  target.pose.position.z = 0.35;
  target.pose.orientation.w = 1;

  move_group.setPoseTarget(target);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto result = move_group.plan(plan);

  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Planning failed.");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Planning succeeded. Executing...");
  auto exec_result = move_group.execute(plan);

  if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Execution failed.");
  } else {
    RCLCPP_INFO(node->get_logger(), "Done.");
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
