/*****************************************************************************
@copyright
Copyright (c) 2020-2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef _INPUT_WHEEL_ROS_H_
#define _INPUT_WHEEL_ROS_H_

#ifdef ROS_BASED
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#else
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#endif

#include "wheel_datatype.h"
typedef void(*WheelCallback)(const sensor_wheel* sensorData);

class InputWheelROS
{
public:
   InputWheelROS();
   ~InputWheelROS();
   
   void addCallback(WheelCallback callBack)
   {
	   wheelCallback = callBack;
   }

private:
#ifdef ROS_BASED
   void wheelOdomCallback( const nav_msgs::msg::Odometry::SharedPtr msg );
   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_sub;
#else
   void wheelOdomCallback( const nav_msgs::Odometry::ConstPtr msg );
   ros::Subscriber wheel_sub;
#endif
   WheelCallback wheelCallback;
};

#endif
