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

#define SAMPLE_RATE_MAX 1000

class ImuNode: public rclcpp::Node
{
public:
    ImuNode() : Node("imu_node")
    {
    }
    ~ImuNode();
    bool Init();
private:
    void PublishCallback();

    int _sample_rate;
    ImuClient _imu_client;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _publisher;
    rclcpp::TimerBase::SharedPtr _timer;
};

ImuNode::~ImuNode()
{
    RCLCPP_INFO(this->get_logger(), "stopping...");
    _imu_client.SendMsgStop(ACCEL_TYPE);
    _imu_client.SendMsgStop(GYRO_TYPE);
}

bool ImuNode::Init()
{
    _sample_rate = this->declare_parameter("sample_rate", 200);
    if (_sample_rate > SAMPLE_RATE_MAX || _sample_rate < 1) {
        RCLCPP_ERROR(this->get_logger(), "sample_rate must from 1 to 1000");
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "sample_rate: %d", _sample_rate);

    if (!_imu_client.InitMmap()) {
        RCLCPP_ERROR(this->get_logger(), "init mmap failed");
        return false;
    }

    if (!_imu_client.ConnectServer()) {
        RCLCPP_ERROR(this->get_logger(), "connect imud failed");
        return false;
    }

    if (!_imu_client.SendMsgConfigRate(_sample_rate)) {
        RCLCPP_ERROR(this->get_logger(), "send CONFIG_RATE to imud failed");
        return false;
    }

    if (!_imu_client.SendMsgStart(ACCEL_TYPE)) {
        RCLCPP_ERROR(this->get_logger(), "send START accel to imud failed");
        return false;
    }

    if (!_imu_client.SendMsgStart(GYRO_TYPE)) {
        RCLCPP_ERROR(this->get_logger(), "send START gyro to imud failed");
        return false;
    }

    _publisher = this->create_publisher<sensor_msgs::msg::Imu>("imu", 30);
    _timer = this->create_wall_timer(
        std::chrono::microseconds(1000000 / _sample_rate),
        std::bind(&ImuNode::PublishCallback,this)
    );

    return true;
}

void ImuNode::PublishCallback()
{
    struct imu_pack_dsp imu_data;
    if (!_imu_client.GetImuData(&imu_data)) {
        RCLCPP_ERROR(this->get_logger(), "get imu data failed");
        return;
    }
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp          = rclcpp::Clock().now();
    imu_msg.linear_acceleration.x = imu_data.acceloration_x;
    imu_msg.linear_acceleration.y = imu_data.acceloration_y;
    imu_msg.linear_acceleration.z = imu_data.acceloration_z;
    imu_msg.angular_velocity.x    = imu_data.angular_velocity_x;
    imu_msg.angular_velocity.y    = imu_data.angular_velocity_y;
    imu_msg.angular_velocity.z    = imu_data.angular_velocity_z;
    _publisher->publish(imu_msg);

    RCLCPP_DEBUG(this->get_logger(), "get imu data <%f,%f,%f,%f,%f,%f>",
        imu_data.acceloration_x,
        imu_data.acceloration_y,
        imu_data.acceloration_z,
        imu_data.angular_velocity_x,
        imu_data.angular_velocity_y,
        imu_data.angular_velocity_z
    );
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuNode>();
    if (!node->Init()) {
        RCLCPP_ERROR(node->get_logger(), "init failed");
    } else {
        RCLCPP_INFO(node->get_logger(), "running...");
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
}
