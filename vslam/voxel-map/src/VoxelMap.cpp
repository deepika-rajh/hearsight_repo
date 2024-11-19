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

#include "VMSystem.h"
#include "InputRGBDCameraROS2.h"
#include "ParseSensorParam.h"
#include "InputWheelROS.h"
#include "vm_apicheck.hpp"

//ROS2 common headers
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <image_transport/image_transport.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cv_bridge/cv_bridge.h>

bool debugLevel = 0;
int RV_LOG_LEVEL = 1;
bool RV_STDERR_LOGGING = true;

#ifndef ROS_BASED
#define ROS_BASED
#endif

static char *helpMsg =
      "mv_vwslam \n"
      "Usage: mv_vwslam [-options]\n"
      "-c : set configuration files path, default path is /usr/share/mono-vslam/ \n"
      "-o : set output files path, default path is /usr/share/mono-vslam/vwslam/ \n"
      "-d : set vslam debug level: enable debug info(1), disable debug info(0) \n"
      "-v : get vslam app version \n"
      "-h : print help msg\n";

rclcpp::Node::SharedPtr g_node = nullptr;
image_transport::Publisher    gray_pub;
image_transport::Publisher    depth_pub;
image_transport::Publisher    labeled_img_pub;
image_transport::Publisher    occupancy_img_pub;
rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_pub = nullptr;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr raw_pose_pub = nullptr;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr robot_pose_pub = nullptr;
rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub = nullptr;

int main( int argc, char** argv )
{
   std::string sensorSetting = std::string( "/usr/share/mono-vslam/" );

   std::string algSetting="/usr/share/mono-vslam/Configuration/vm.cfg";

   if (argc == 2)
   {
	   algSetting = std::string(argv[1]);
   }

   rclcpp::init(argc, argv);
   g_node = rclcpp::Node::make_shared("voxel_map");

   occupancy_img_pub = image_transport::create_publisher(g_node.get(), OCCUPANCY_IMG_NAME);

#ifdef ARM_BASED
   //add log for ARM platform to check boot time
   system("echo VM Start Initialization > /dev/kmsg");
#endif

   //start VSLAM system
   std::string cameraSettingFile;
   GetCameraSettingFile( sensorSetting, "Configuration/vslam.cfg", cameraSettingFile);
   std::shared_ptr<CameraInterface> inputCamera = std::make_shared<InputRGBDCameraROS2>(g_node, sensorSetting+cameraSettingFile);
   std::shared_ptr<VMSystem> sys = VMSystem::Initialize(algSetting, inputCamera);

   sys->Run();

   //wait to quit
   //sys->Spin();
   rclcpp::spin( g_node );

   //stop VSLAM
   sys->Quit();
   sys->deinit();
#ifdef ROS_BASED
   sys->state_sub = nullptr;
   sys->cameraInMapPose_sub = nullptr;
#endif
   sys = nullptr;
   printf("vm application exits\n");

   // labeled_img_pub.shutdown();
   occupancy_img_pub.shutdown();
   rclcpp::shutdown();

   g_node = nullptr;
   printf("release ros node done\n");
   return 0;
}

void showOccupancyImg(const cv::Mat & gridImage)
{
    sensor_msgs::msg::Image::SharedPtr img;
    img = cv_bridge::CvImage(
         std_msgs::msg::Header(), sensor_msgs::image_encodings::MONO8, gridImage ).toImageMsg();

    rclcpp::Clock ros_clock( RCL_ROS_TIME );

    img->width = gridImage.cols;
    img->height = gridImage.rows;
    img->is_bigendian = false;
    img->step = gridImage.cols;
    img->header.frame_id = "occupancy_image";
    img->header.stamp = ros_clock.now();
    occupancy_img_pub.publish( img );
}

void stopExternalElements()
{
	rclcpp::shutdown();
}

void showDepthImage(const cv::Mat& /*colorsMap*/)
{

}
