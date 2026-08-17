#include <rclcpp/rclcpp.hpp>

#include "semantic_detection/ros/YoloNode.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<semantic_detection::YoloNode>();
    rclcpp::spin(node);
    node->printSummary();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("yolo_node"), "semantic_detection: startup failed: %s",
                e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
