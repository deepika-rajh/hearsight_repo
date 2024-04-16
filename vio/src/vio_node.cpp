/*****************************************************************************
Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
All rights reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "vio_component.hpp"

int main(int argc, char* argv[])
{
   rclcpp::init(argc, argv);
   rclcpp::NodeOptions options;
   rclcpp::executors::SingleThreadedExecutor exec;
   auto node = std::make_shared<qrb_ros_vio::VioComponent>(options);
   exec.add_node(node);
   exec.spin();

   rclcpp::shutdown();

   return 0;
}
