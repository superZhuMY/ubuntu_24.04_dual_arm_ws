// Fault-injected test executable. Production keeps the normal ROS transport.
#include <atomic>
#include <memory>
#include <stdexcept>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "double_arm_harvest_execution/dual_harvest_executor.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  const auto injected = std::make_shared<std::atomic<bool>>(false);
  double_arm_harvest_execution::TrajectoryDispatcher::SendGoal send =
    [injected](bool left, const auto & client, const auto & goal, const auto & callbacks) {
      (void)client->async_send_goal(goal, callbacks);
      if (!left && !injected->exchange(true)) {
        throw std::runtime_error("injected error after right submission");
      }
    };
  auto node = std::make_shared<double_arm_harvest_execution::TaskExecutor>(options, send);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 6);
  executor.add_node(node);
  std::thread spinner([&executor] {executor.spin();});
  node->init_move_groups();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}
