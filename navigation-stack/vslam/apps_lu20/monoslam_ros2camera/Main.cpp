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
#include "InputMonoCameraROS2.h"
#include "ParseSensorParam.h"
#include "InputWheelROS.h"
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
   std::string sensorSetting = std::string( "/data/misc/vwslam/" );
   std::string algSetting="/data/misc/vwslam/Configuration/monoSlam.cfg";
   std::string output="/data/vwslam/";

   if (argc == 4)
   {
	   sensorSetting = std::string(argv[1]);
	   algSetting = std::string(argv[2]);
	   output = std::string(argv[3]);
   }

   rclcpp::init(argc, argv);
   signal(SIGINT, INT_handler);
   g_node = rclcpp::Node::make_shared("dvslam_ros2camera");

   labeled_img_pub = image_transport::create_publisher(g_node.get(), "vslam/labeled_img");
   raw_pose_pub = g_node.get()->create_publisher<nav_msgs::msg::Odometry>("vslam_odom_raw", 5);
   robot_pose_pub = g_node.get()->create_publisher<nav_msgs::msg::Odometry>("robot_odom", 5);
   imu_pub = g_node.get()->create_publisher<sensor_msgs::msg::Imu>("sensor_imu", 30);

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
   std::shared_ptr<CameraInterface> inputCamera = std::make_shared<InputMonoCameraROS2>(g_node, sensorSetting+cameraSettingFile);
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

   labeled_img_pub.shutdown();
   rclcpp::shutdown();
   raw_pose_pub = nullptr;
   robot_pose_pub = nullptr;
   cam_info_pub = nullptr;
   imu_pub = nullptr;
   g_node = nullptr;
   printf("release ros node done\n");
   return 0;
}
