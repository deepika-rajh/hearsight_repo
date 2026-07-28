/*****************************************************************************
@copyright
Copyright (c) 2023-2024 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include <iostream>
#include "InputStereoCameraROS2.h"

// for ROS2
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <image_transport/image_transport.hpp>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"

rclcpp::Node::SharedPtr g_node = nullptr;
image_transport::Publisher color_pub;
image_transport::Publisher labeled_img_pub;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr raw_pose_pub = nullptr;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr robot_pose_pub = nullptr;
rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub = nullptr;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub = nullptr;
std::shared_ptr<tf2_ros::TransformListener> tf_listener = nullptr;
std::shared_ptr<tf2_ros::Buffer> tf_buffer = nullptr;
std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster = nullptr;

#include <VSLAMSystem.h>
#ifdef WHEEL_SUPPORTED
#include <InputWheelROS.h>
#include <VSLAMWheel.h>
#endif
#ifndef IMU_SUPPORTED
#include "InputIMUROS2.h"
#endif

#include "ParseSensorParam.h"

int RV_LOG_LEVEL = 1;
bool RV_STDERR_LOGGING = true;

static void INT_handler(int sig)
{
    printf("signal received to stop\n");
    rclcpp::shutdown();
}

std::string GetStringParam(const std::string &param_name)
{
    rclcpp::Parameter str_param = g_node.get()->get_parameter(param_name);
    return str_param.as_string();
}

int main(int argc, char **argv)
{
    // stdout is fully block-buffered (not line-buffered) when it's not a real
    // TTY - the case when launched via `ros2 launch`. Without this, printf()
    // output (including any additional diagnostics the closed-source engine
    // prints about feature detection/triangulation) sits in the buffer and
    // never reaches the console/log. Same fix already applied in
    // depth-vslam/src/DepthvSLAM.cpp.
    setvbuf(stdout, NULL, _IOLBF, 0);

    std::string algSetting = "/opt/qcom/qirf-sdk/data/misc/vwslam/Configuration/stereoSlam.cfg";
    std::string sensorSetting = "/opt/qcom/qirf-sdk/data/misc/vwslam/";
    std::string output = "/opt/qcom/qirf-sdk/data/vwslam/";
    std::string vslam_config_file = "Configuration/robot.cfg";


    rclcpp::init(argc, argv);
    signal(SIGINT, INT_handler);

    g_node = rclcpp::Node::make_shared("stereo_vslam");

    raw_pose_pub = g_node.get()->create_publisher<nav_msgs::msg::Odometry>("vslam_odom_raw", 5);
    robot_pose_pub = g_node.get()->create_publisher<nav_msgs::msg::Odometry>("robot_odom", 5);
    imu_pub = g_node.get()->create_publisher<sensor_msgs::msg::Imu>("sensor_imu", 30);
    point_cloud_pub = g_node.get()->create_publisher<sensor_msgs::msg::PointCloud2>("vslam/point_cloud", 10);
    labeled_img_pub = image_transport::create_publisher(g_node.get(), "vslam/labeled_image");
    tf_buffer = std::make_shared<tf2_ros::Buffer>(g_node->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*g_node);

#ifdef WHEEL_SUPPORTED
    InputWheelROS wheel;
    wheel.addCallback(VSLAMWheel::wheelCallback);
#endif
#ifndef IMU_SUPPORTED
    // No on-board IMU on this rig: feed the RealSense D455's IMU over ROS2
    // instead (see depth-vslam's DepthvSLAM.cpp for the same pattern).
    // Subscribes to the node-relative topic "imu", remapped via input_imu_topic.
    InputIMUROS2 imu(g_node.get());
    imu.addCallback(VSLAMSystem::addIMU);
#endif

    g_node->declare_parameter("alg_setting", algSetting);
    algSetting = g_node->get_parameter("alg_setting").as_string();
    g_node->declare_parameter("sensor_setting", sensorSetting);
    sensorSetting = g_node->get_parameter("sensor_setting").as_string();
    g_node->declare_parameter("output_file", output);
    output = g_node->get_parameter("output_file").as_string();
    g_node->declare_parameter("vslam_config_file", vslam_config_file);
    vslam_config_file = g_node->get_parameter("vslam_config_file").as_string();

    // camera_yaml: explicit calibration override (absolute, or relative to
    // sensor_setting). Empty => use the Stereo/Camera line from vslam_config_file;
    // if that is missing/unreadable the stereo adapter auto-detects resolution +
    // intrinsics from the live /left/camera_info and /right/camera_info topics.
    std::string cameraYaml = "";
    g_node->declare_parameter("camera_yaml", cameraYaml);
    cameraYaml = g_node->get_parameter("camera_yaml").as_string();

    // start VSLAM system
    std::string cameraSettingFile;
    if (!cameraYaml.empty())
    {
        cameraSettingFile = cameraYaml;
        printf("[stereo-vslam] camera calibration from 'camera_yaml' param: %s\n", cameraSettingFile.c_str());
    }
    else
    {
        GetCameraSettingFile(sensorSetting, vslam_config_file, cameraSettingFile);
        printf("[stereo-vslam] camera calibration from %s Stereo/Camera line: %s\n",
               vslam_config_file.c_str(), cameraSettingFile.c_str());
    }

    // Absolute path -> use as-is; otherwise resolve relative to sensor_setting.
    std::string cameraCalibPath = (!cameraSettingFile.empty() && cameraSettingFile[0] == '/')
                                  ? cameraSettingFile : sensorSetting + cameraSettingFile;
    std::shared_ptr<CameraInterface> inputCamera = std::make_shared<InputStereoCameraROS2>(g_node, cameraCalibPath);
    ParseSensorParam(sensorSetting, vslam_config_file, VSLAMSystem::wheelConfiguration, VSLAMSystem::imuConfiguration, VSLAMSystem::targetImage);
    std::shared_ptr<VSLAMSystem> sys = VSLAMSystem::Initialize(algSetting, output, inputCamera, false);
    sys->Run();

    // wait to quit
    rclcpp::spin(g_node);

    // stop VSLAM
    sys->Quit();
    sys->deinit();
    sys = nullptr;
    printf("vslam application exits\n");
    fflush(stdout);

    labeled_img_pub.shutdown();
    rclcpp::shutdown();
    raw_pose_pub = nullptr;
    robot_pose_pub = nullptr;
    imu_pub = nullptr;
    point_cloud_pub = nullptr;
    g_node = nullptr;
    printf("release ros node done\n");
    return 0;
}
