/*****************************************************************************
@copyright
Copyright (c) 2023-2024 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>

#define ODOM_RAW_NAME "vslam_odom_raw"
#define ROBOT_ODOM_NAME "robot_odom"
#define ODOM_TYPE nav_msgs::msg::Odometry

#define IMU_NAME "sensor_imu"
#define IMU_TYPE sensor_msgs::msg::Imu
