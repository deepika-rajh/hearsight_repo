/*****************************************************************************
@copyright
Copyright (c) 2020-2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef _INPUT_WHEEL_ROS_H_
#define _INPUT_WHEEL_ROS_H_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

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
   void wheelOdomCallback( const nav_msgs::msg::Odometry::SharedPtr msg );
   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_sub;

   WheelCallback wheelCallback;
};

#endif
