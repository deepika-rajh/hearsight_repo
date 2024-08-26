/*****************************************************************************
Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
All rights reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/
#include "vio_component.hpp"
#include "InputOv9282ROS2.h"
#include "vio_apicheck.hpp"

rclcpp::Publisher<ODOM_TYPE>::SharedPtr raw_pose_pub = nullptr;
image_transport::Publisher    labeled_img_pub;

namespace qrb_ros_vio
{
VioComponent::VioComponent(const rclcpp::NodeOptions& options) : Node("vio_node", options), imu(this)
{
    long boot_time = (rclcpp::Clock().now()).nanoseconds();

    labeled_img_pub = image_transport::create_publisher(this, LABEL_IMG_NAME);
    raw_pose_pub = create_publisher<ODOM_TYPE>(ODOM_RAW_NAME, 5);

    imu.addCallback(VISLAMSystem::addIMU);

    std::string sensorPath = this->declare_parameter<std::string>("sensor_file_path", "/opt/qcom/qirf-sdk/data/misc/vwslam/");
    std::string outputPath = this->declare_parameter<std::string>("output_path", "/opt/qcom/qirf-sdk/data/vwslam/");
    std::string algConfFile = this->declare_parameter<std::string>("algorithm_file", "Configuration/vislam.cfg");

    std::string cameraSettingFile;
    GetCameraSettingFile(sensorPath, algConfFile, cameraSettingFile);
    std::shared_ptr<CameraInterface> inputCamera = std::make_shared<InputOv9282ROS2>(*this, sensorPath + cameraSettingFile);

    rvTargetImage target;
    ParseSensorParam(sensorPath, algConfFile, VISLAMSystem::wheelConfiguration, VISLAMSystem::imuConfiguration, target);
    sys = VISLAMSystemROS2::Initialize(sensorPath + algConfFile, outputPath, inputCamera, *this);

    sys->Run();

    long start_time = (rclcpp::Clock().now()).nanoseconds();

    printf("VIO boot up time is : %ld ns\n", start_time - boot_time);
}

VioComponent::~VioComponent()
{
    sys->Quit();
    sys->deinit();
    sys = nullptr;
    printf("vislam application exits\n");
    RCLCPP_DEBUG(this->get_logger(), "vislam application exits\n");

    fflush(stdout);

    labeled_img_pub.shutdown();
    raw_pose_pub = nullptr;
}
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(qrb_ros_vio::VioComponent)
