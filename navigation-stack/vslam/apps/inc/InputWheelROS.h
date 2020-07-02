/*****************************************************************************
@copyright
Copyright (c) 2020 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef _INPUT_WHEEL_ROS_H_
#define _INPUT_WHEEL_ROS_H_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

class InputWheelROS
{
public:
   InputWheelROS();
   ~InputWheelROS();

private:
   void wheelOdomCallback( const nav_msgs::msg::Odometry::SharedPtr msg );
   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_sub;

};
#endif
