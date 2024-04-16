/*****************************************************************************
Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
All rights reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/
#ifndef _VIO_COMPONENT_HPP_
#define _VIO_COMPONENT_HPP_

#include <thread>
#include <inttypes.h>
#include <signal.h>
#include <functional>
#include <string.h>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>

#include "memory.h"
#include "math.h"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <opencv2/opencv.hpp>

#include "VISLAMSystemROS2.h"
#include "InputOv9282ROS2.h"
#include "InputIMUROS2.h"
#include "ParseSensorParam.h"

namespace qrb_ros_vio
{
class VioComponent : public rclcpp::Node
{
    public:
        explicit VioComponent(const rclcpp::NodeOptions& options);
        ~VioComponent();

    private:
	    std::shared_ptr<CameraInterface> inputCamera;
		InputIMUROS2 imu;
		std::shared_ptr<VISLAMSystem> sys;
};
}

#endif
