/*****************************************************************************
Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
All rights reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include <memory>
#include "InputIMUROS2.h"

using std::placeholders::_1;

InputIMUROS2::InputIMUROS2(rclcpp::Node * g_node)
{
    auto my_callback_group = g_node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
    sub_options.callback_group = my_callback_group;

    imu_sub = g_node->create_subscription<sensor_msgs::msg::Imu>( "imu", 10,
                                   std::bind(&InputIMUROS2::imuROSCallback, this,  _1), sub_options);
}

InputIMUROS2::~InputIMUROS2()
{

}

void InputIMUROS2::imuROSCallback( const sensor_msgs::msg::Imu::SharedPtr msg )
{
   rclcpp::Time t = msg->header.stamp;
   int64_t timestamp = t.nanoseconds();

   angularVelocity[0] = (float)msg->angular_velocity.x;
   angularVelocity[1] = (float)msg->angular_velocity.y;
   angularVelocity[2] = (float)msg->angular_velocity.z;
   linearAcceleration[0] = (float)msg->linear_acceleration.x;
   linearAcceleration[1] = (float)msg->linear_acceleration.y;
   linearAcceleration[2] = (float)msg->linear_acceleration.z;
   imuCallback(linearAcceleration, angularVelocity, timestamp);
}
