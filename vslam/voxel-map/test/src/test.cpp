/*****************************************************************************
@copyright
Copyright (c) 2023-2024 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/
#include "gtest/gtest.h"
#include "vm_apicheck.hpp"

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
    auto pub = test_node->create_publisher<OCCUPANCY_IMG_TYPE>(OCCUPANCY_IMG_NAME, 10);
    auto sub = test_node->create_subscription<sensor_msgs::msg::Image>(
      "vm/occupancy_img", 10,
      [&h](const sensor_msgs::msg::Image::SharedPtr msg) {
          h = 1U;
      });

    EXPECT_EQ(pub->get_subscription_count(), 1U);
    EXPECT_EQ(sub->get_publisher_count(), 1U);

    auto message = sensor_msgs::msg::Image();

    pub->publish(message);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    rclcpp::spin_some(test_node);

    EXPECT_EQ(h, 1U);

    pub.reset();
    sub.reset();
    test_node.reset();
}
