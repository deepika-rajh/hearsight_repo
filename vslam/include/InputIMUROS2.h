/*****************************************************************************
Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
All rights reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef _INPUT_IMU_ROS2_H_
#define _INPUT_IMU_ROS2_H_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

typedef void(*IMUCallback)(const float linearAcceleration[3], const float angularVelocity[3], int64_t timestamp);


class InputIMUROS2
{
public:
   InputIMUROS2(rclcpp::Node * g_node);
   ~InputIMUROS2();

   void addCallback(IMUCallback callBack)
   {
	   imuCallback = callBack;
   }

private:
   void imuROSCallback( const sensor_msgs::msg::Imu::SharedPtr msg );
   rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
   IMUCallback imuCallback;
   float linearAcceleration[3], angularVelocity[3];
};

#endif
