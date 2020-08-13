/*
 * Copyright (c) 2020 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <chrono>
#include <functional>
#include "imu-ros2node/imu_client.hpp"

using namespace std::chrono_literals;

class ImuTest: public rclcpp::Node
{
public:
    ImuNode() : Node("imu_test")
    {
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuTest>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
