/*****************************************************************************
@copyright
Copyright (c) 2023-2024 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#define GOAL_NAME "/goal_pose"
#define GOAL_TYPE geometry_msgs::msg::PoseStamped
