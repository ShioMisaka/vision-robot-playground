#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include "robot_tasks/grasp_task_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto task_node = robot_tasks::GraspTaskNode::create();
  if (!task_node) {
    rclcpp::shutdown();
    return 1;
  }

  // VisionProcessorNode 和 RobotClient 是独立的 rclcpp::Node，
  // 需要加入 MultiThreadedExecutor 才能处理图像订阅和服务回调。
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(task_node);
  for (const auto& sub_node : task_node->get_sub_nodes()) {
    executor.add_node(sub_node);
  }

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
