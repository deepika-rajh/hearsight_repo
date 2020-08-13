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

class ImuNode: public rclcpp::Node
{
public:
    ImuNode() : Node("imu_node")
    {
        _imu_client.InitMmap();
        _publisher = this->create_publisher<sensor_msgs::msg::Imu>("imu", 30);
        this->create_wall_timer(500ms, std::bind(&ImuNode::PublishCallback, this));
    }
private:
    void PublishCallback();

    ImuClient _imu_client;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _publisher;
};

void ImuNode::PublishCallback()
{
    struct imu_pack_dsp imu_data;
    if (!_imu_client.GetImuData(&imu_data))
        return;
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = rclcpp::Clock().now();
    imu_msg.linear_acceleration.x = imu_data.acceloration_x;
    imu_msg.linear_acceleration.y = imu_data.acceloration_y;
    imu_msg.linear_acceleration.z = imu_data.acceloration_z;
    imu_msg.angular_velocity.x = imu_data.angular_velocity_x;
    imu_msg.angular_velocity.y = imu_data.angular_velocity_y;
    imu_msg.angular_velocity.z = imu_data.angular_velocity_z;
    _publisher->publish(imu_msg);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
