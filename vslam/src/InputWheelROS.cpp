/*****************************************************************************
@copyright
Copyright (c) 2020-2023 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include <memory>
#include "InputWheelROS.h"
#include "rvQueue.h"
#include "wheel_datatype.h"
using std::placeholders::_1;

extern queue_mt<sensor_wheel> wheelDataArray;
extern rclcpp::Node::SharedPtr g_node;
InputWheelROS::InputWheelROS()
{
   wheel_sub = g_node->create_subscription<nav_msgs::msg::Odometry>( "odom", 10,
                                   std::bind(&InputWheelROS::wheelOdomCallback, this,  _1));
}

InputWheelROS::~InputWheelROS()
{

}

void InputWheelROS::wheelOdomCallback( const nav_msgs::msg::Odometry::SharedPtr msg )
{
   //printf("*** get wheel data\n");
   sensor_wheel wheelodom;
   rclcpp::Time t = msg->header.stamp;
   wheelodom.timestamp = t.nanoseconds();

   wheelodom.location[0] = msg->pose.pose.position.x;
   wheelodom.location[1] = msg->pose.pose.position.y;
   wheelodom.location[2] = msg->pose.pose.position.z;
   wheelodom.direction[0] = msg->pose.pose.orientation.x;
   wheelodom.direction[1] = msg->pose.pose.orientation.y;
   wheelodom.direction[2] = msg->pose.pose.orientation.z;
   wheelodom.direction[3] = msg->pose.pose.orientation.w;
   wheelodom.linear_velocity = msg->twist.twist.linear.x;
   wheelodom.angular_velocity = msg->twist.twist.angular.z;
   wheelCallback(&wheelodom);
}



