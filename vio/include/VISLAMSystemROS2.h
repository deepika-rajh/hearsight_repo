/*****************************************************************************
Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
All rights reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef __VISLAM_SYSTEM_ROS2_H__
#define __VISLAM_SYSTEM_ROS2_H__

#include "VISLAMSystem.h"

class VISLAMSystemROS2: public VISLAMSystem
{
public:
    VISLAMSystemROS2(rclcpp::Node & g_node, std::shared_ptr<CameraInterface>& camera);
    ~VISLAMSystemROS2();
    static std::shared_ptr<VISLAMSystem> Initialize(const std::string& algSetting, const std::string& outputDir,
        std::shared_ptr<CameraInterface> camera, rclcpp::Node & g_node);
    virtual void deinit0();

private:

    rclcpp::Node & node;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub;

    void state_callbackROS(const std_msgs::msg::String::SharedPtr msg) const;
    void pub_camera_raw_pose(const rvVISLAMPose & pose);
};

#endif //__VISLAM_SYSTEM_ROS2_H__
