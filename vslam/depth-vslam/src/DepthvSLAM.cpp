/*****************************************************************************
@copyright
Copyright (c) 2022-2023 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include <stdlib.h>
#include <iostream>
#include <string>
#include <unistd.h>

#include "VSLAMSystem.h"
#include "InputRGBDCameraROS2.h"
#include "ParseSensorParam.h"
#include "InputWheelROS.h"
#include "vslam_apicheck.hpp"
#include <VSLAMWheel.h>

//ROS2 common headers
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <image_transport/image_transport.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

bool debugLevel = 0;
bool RV_STDERR_LOGGING = true;

rclcpp::Node::SharedPtr g_node = nullptr;

image_transport::Publisher    labeled_img_pub;
image_transport::Publisher    occupancy_img_pub;
rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_pub = nullptr;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr raw_pose_pub = nullptr;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr robot_pose_pub = nullptr;
rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub = nullptr;

static void INT_handler (int /*sig*/)
{
    printf("signal received to stop\n");
    rclcpp::shutdown();
}

int main( int argc, char** argv )
{
   std::string sensorSetting = std::string( "/opt/qcom/qirf-sdk/data/misc/vwslam/" );
   std::string algSetting="/opt/qcom/qirf-sdk/data/misc/vwslam/Configuration/rgbdWSlam.cfg";
   std::string output="/opt/qcom/qirf-sdk/data/vwslam/";

   if (argc == 4)
   {
	   sensorSetting = std::string(argv[1]);
	   algSetting = std::string(argv[2]);
	   output = std::string(argv[3]);
   }

   rclcpp::init(argc, argv);
   signal(SIGINT, INT_handler);
   g_node = rclcpp::Node::make_shared("depth_vslam");

   labeled_img_pub = image_transport::create_publisher(g_node.get(), LABEL_IMG_NAME);
   raw_pose_pub = g_node.get()->create_publisher<ODOM_TYPE>(ODOM_RAW_NAME, 5);
   robot_pose_pub = g_node.get()->create_publisher<ODOM_TYPE>(ROBOT_ODOM_NAME, 5);
   imu_pub = g_node.get()->create_publisher<IMU_TYPE>(IMU_NAME, 30);

   char tmp = *(output.end() - 1);
   if( tmp != '/' && tmp != '\\' )
   {
      output = output + '/';
   }

   tmp = *(sensorSetting.end() - 1);
   if( tmp != '/' && tmp != '\\' )
   {
      sensorSetting = sensorSetting + '/';
   }

#ifdef ARM_BASED
   //add log for ARM platform to check boot time
   system("echo vSLAM Start Initialization > /dev/kmsg");
#endif

   std::string cameraSettingFile;
   InputWheelROS wheel;
   wheel.addCallback(VSLAMWheel::wheelCallback);
   //start VSLAM system
   GetCameraSettingFile( sensorSetting, "Configuration/vslam.cfg", cameraSettingFile);
   std::shared_ptr<CameraInterface> inputCamera = std::make_shared<InputRGBDCameraROS2>(g_node, sensorSetting+cameraSettingFile);
   ParseSensorParam(sensorSetting, "Configuration/vslam.cfg", VSLAMSystem::wheelConfiguration, VSLAMSystem::imuConfiguration, VSLAMSystem::targetImage);

   std::shared_ptr<VSLAMSystem> sys = VSLAMSystem::Initialize( algSetting, output, inputCamera, false);

   sys->Run();

   //wait to quit
   sys->Spin();

   //stop VSLAM
   sys->Quit();
   sys->deinit();
   sys->state_sub = nullptr;
   sys = nullptr;
   printf("vslam application exits\n");

   //color_pub.shutdown();
   //gray_pub.shutdown();
   //depth_pub.shutdown();
   labeled_img_pub.shutdown();
   occupancy_img_pub.shutdown();
   rclcpp::shutdown();
   raw_pose_pub = nullptr;
   robot_pose_pub = nullptr;
   cam_info_pub = nullptr;
   imu_pub = nullptr;
   g_node = nullptr;
   printf("release ros node done\n");
   return 0;
}
