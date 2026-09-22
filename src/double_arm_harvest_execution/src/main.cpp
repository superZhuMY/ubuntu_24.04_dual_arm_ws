// Copyright 2026 zmy

#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "double_arm_harvest_execution/dual_harvest_executor.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // MoveGroupInterface expects robot_description / robot_description_semantic
  // / robot_description_kinematics as parameter overrides.
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<double_arm_harvest_execution::DualHarvestExecutor>(options);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 6);
  executor.add_node(node);

  // Keep the node spinning while MoveGroupInterface blocks on move_group
  // services during construction.
  std::thread spinner([&executor]() {executor.spin();});
  node->init_move_groups();

  spinner.join();
  rclcpp::shutdown();
  return 0;
}
