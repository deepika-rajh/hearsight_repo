/*****************************************************************************
@copyright
Copyright (c) 2023-2024 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/
#include "gtest/gtest.h"
#include "vio_apicheck.hpp"

class NodeTestSuite : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
    }

    void TearDown() override {
        rclcpp::shutdown();
    }
};

TEST_F(NodeTestSuite, RosMessageTest1)
{
    rclcpp::Node::SharedPtr test_node = std::make_shared<rclcpp::Node>("test_node");

    uint16_t h = 0;
    auto pub = test_node->create_publisher<ODOM_TYPE>(ODOM_RAW_NAME, 10);
    auto sub = test_node->create_subscription<nav_msgs::msg::Odometry>(
      "vslam_odom_raw", 10, 
      [&h](const nav_msgs::msg::Odometry::SharedPtr msg) {
          h = 1U;
      });

    EXPECT_EQ(pub->get_subscription_count(), 1U);
    EXPECT_EQ(sub->get_publisher_count(), 1U);

    auto message = nav_msgs::msg::Odometry();

    pub->publish(message);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    rclcpp::spin_some(test_node);

    EXPECT_EQ(h, 1U);

    pub.reset();
    sub.reset();
    test_node.reset();
}

TEST_F(NodeTestSuite, RosMessageTest2)
{
    rclcpp::Node::SharedPtr test_node = std::make_shared<rclcpp::Node>("test_node");

    uint16_t h = 0;
    auto pub = test_node->create_publisher<ODOM_TYPE>(ODOM_NAME, 10);
    auto sub = test_node->create_subscription<nav_msgs::msg::Odometry>(
      "robot_odom", 10, 
      [&h](const nav_msgs::msg::Odometry::SharedPtr msg) {
          h = 1U;
      });

    EXPECT_EQ(pub->get_subscription_count(), 1U);
    EXPECT_EQ(sub->get_publisher_count(), 1U);

    auto message = nav_msgs::msg::Odometry();
    pub->publish(message);

    std::this_thread::sleep_for(std::chrono::seconds(1));
    rclcpp::spin_some(test_node);

    EXPECT_EQ(h, 1U);

    pub.reset();
    sub.reset();
    test_node.reset();
}

TEST_F(NodeTestSuite, RosMessageTest3)
{
    rclcpp::Node::SharedPtr test_node = std::make_shared<rclcpp::Node>("test_node");

    uint16_t h = 0;
    auto pub = test_node->create_publisher<IMU_TYPE>(IMU_NAME, 10);
    auto sub = test_node->create_subscription<sensor_msgs::msg::Imu>(
      "sensor_imu", 10, 
      [&h](const sensor_msgs::msg::Imu::SharedPtr msg) {
          h = 1U;
      });

    EXPECT_EQ(pub->get_subscription_count(), 1U);
    EXPECT_EQ(sub->get_publisher_count(), 1U);

    auto message = sensor_msgs::msg::Imu();
    pub->publish(message);

    std::this_thread::sleep_for(std::chrono::seconds(1));
    rclcpp::spin_some(test_node);

    EXPECT_EQ(h, 1U);

    pub.reset();
    sub.reset();
    test_node.reset();
}
