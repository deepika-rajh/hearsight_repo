/*****************************************************************************
Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
All rights reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef __STDC_FORMAT_MACROS
#define __STDC_FORMAT_MACROS
#endif
#include <inttypes.h>

#include <thread>
#include <signal.h>
#include <functional>
#include <string.h>
#include <math.h>

#include "VISLAMSystemROS2.h"
#include "SystemTime.h"

#include <sstream>
#include <fstream>

#include <rvQueue.h>

#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>

using std::placeholders::_1;
extern rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr raw_pose_pub;

void VISLAMSystemROS2::pub_camera_raw_pose(const rvVISLAMPose & pose)
{
  auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();

  odom_msg->header.frame_id = "odom";
  odom_msg->child_frame_id  = "camera_link";
  odom_msg->header.stamp = rclcpp::Time(pose.time, RCL_ROS_TIME);

  odom_msg->pose.pose.position.x = pose.bodyPose.matrix[0][3];
  odom_msg->pose.pose.position.y = pose.bodyPose.matrix[1][3];
  odom_msg->pose.pose.position.z = pose.bodyPose.matrix[2][3];

  float qw, qx, qy, qz;
  Matrix2Quaternion(pose.bodyPose.matrix, qw, qx, qy, qz);

  odom_msg->pose.pose.orientation.x =  qx;
  odom_msg->pose.pose.orientation.y =  qy;
  odom_msg->pose.pose.orientation.z =  qz;
  odom_msg->pose.pose.orientation.w =  qw;

  odom_msg->twist.twist.linear.x  = 0;
  odom_msg->twist.twist.angular.z = 0;

  raw_pose_pub->publish(std::move(odom_msg));
}

VISLAMSystemROS2::VISLAMSystemROS2(rclcpp::Node & g_node_, std::shared_ptr<CameraInterface>& camera):VISLAMSystem(camera), node(g_node_)
{
   state_sub = node.create_subscription<std_msgs::msg::String>( "vslam_state", 10,
        std::bind(&VISLAMSystemROS2::state_callbackROS, this,  _1));
}

std::shared_ptr<VISLAMSystem>
VISLAMSystemROS2::Initialize(const std::string& algSetting, const std::string& outputDir,
       std::shared_ptr<CameraInterface> camera, rclcpp::Node & g_node)
{
   if( t.get() == nullptr )
   {
      t = std::make_shared<VISLAMSystemROS2>(g_node, camera);
      VISLAMSystem::Initialize(algSetting, outputDir);
      signal(SIGINT, shutdown);
   }
   return t;
}

void VISLAMSystemROS2::deinit0()
{
    state_sub = nullptr;
    VISLAMSystem::deinit0();
}

VISLAMSystemROS2::~VISLAMSystemROS2()
{
}

void VISLAMSystemROS2::state_callbackROS(const std_msgs::msg::String::SharedPtr msg) const
{
    state_callback(msg->data);
}
