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
#include "vslam_apicheck.hpp"
#ifdef WHEEL_SUPPORTED
#include "InputWheelROS.h"
#include <VSLAMWheel.h>
#endif
#ifndef IMU_SUPPORTED
#include "InputIMUROS2.h"
#endif

//ROS2 common headers
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <image_transport/image_transport.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

bool debugLevel = 0;
bool RV_STDERR_LOGGING = true;

rclcpp::Node::SharedPtr g_node = nullptr;

image_transport::Publisher    labeled_img_pub;
image_transport::Publisher    occupancy_img_pub;
rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_pub = nullptr;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr raw_pose_pub = nullptr;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr robot_pose_pub = nullptr;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub = nullptr;
rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub = nullptr;

static void INT_handler (int /*sig*/)
{
    printf("signal received to stop\n");
    rclcpp::shutdown();
}

int main( int argc, char** argv )
{
   // stdout is fully block-buffered (not line-buffered) when it's not a real
   // TTY - which is the case when launched via `ros2 launch`. Without this,
   // printf() output sits in the buffer and never reaches the console/log,
   // while std::cout with std::endl (which flushes) still shows up.
   setvbuf(stdout, NULL, _IOLBF, 0);

   std::string sensorSetting = std::string( "/usr/share/mono-vslam/" );
   std::string algSetting="/usr/share/mono-vslam/Configuration/rgbdWSlam.cfg";
   std::string output="/usr/share/mono-vslam/vwslam/";
   std::string cameraYaml = "";

   rclcpp::init(argc, argv);
   signal(SIGINT, INT_handler);
   g_node = rclcpp::Node::make_shared("depth_vslam");

   // Make config paths and camera calibration configurable at run time so the
   // node is not tied to a hardcoded resolution / RealSense-only calibration.
   // Priority: positional args (legacy) > ROS parameters > built-in defaults.
   //   sensor_setting : directory that holds Configuration/vslam.cfg etc.
   //   alg_setting    : SLAM algorithm .cfg
   //   output_file    : map/log output directory
   //   camera_yaml    : absolute or sensor_setting-relative path to an OpenCV
   //                    (%YAML:1.0) camera calibration file. Leave empty to take
   //                    the calibration from vslam.cfg's "Camera" line; if that
   //                    is missing/unreadable the node auto-detects resolution +
   //                    intrinsics from the live /camera/color/camera_info topic.
   sensorSetting = g_node->declare_parameter<std::string>("sensor_setting", sensorSetting);
   algSetting    = g_node->declare_parameter<std::string>("alg_setting", algSetting);
   output        = g_node->declare_parameter<std::string>("output_file", output);
   cameraYaml    = g_node->declare_parameter<std::string>("camera_yaml", cameraYaml);

   if (argc == 4)
   {
	   sensorSetting = std::string(argv[1]);
	   algSetting = std::string(argv[2]);
	   output = std::string(argv[3]);
   }

   labeled_img_pub = image_transport::create_publisher(g_node.get(), LABEL_IMG_NAME);
   raw_pose_pub = g_node.get()->create_publisher<ODOM_TYPE>(ODOM_RAW_NAME, 5);
   robot_pose_pub = g_node.get()->create_publisher<ODOM_TYPE>(ROBOT_ODOM_NAME, 5);
   imu_pub = g_node.get()->create_publisher<IMU_TYPE>(IMU_NAME, 30);
   point_cloud_pub = g_node.get()->create_publisher<sensor_msgs::msg::PointCloud2>("vslam/point_cloud", 10);

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
#ifdef WHEEL_SUPPORTED
   InputWheelROS wheel;
   wheel.addCallback(VSLAMWheel::wheelCallback);
#endif
#ifndef IMU_SUPPORTED
   // No on-board IMU on this rig: feed the RealSense D455's IMU over ROS2
   // instead of the on-board sensor-client path (see depth-vslam/CMakeLists.txt).
   // Subscribes to the node-relative topic "imu", remapped to /camera/imu by the
   // launch file. The D455 must be launched with unite_imu_method:=2 (or 1) so
   // accel+gyro arrive combined on one topic instead of two separate ones.
   InputIMUROS2 imu(g_node.get());
   imu.addCallback(VSLAMSystem::addIMU);
#endif
   //start VSLAM system
   if( !cameraYaml.empty() )
   {
      cameraSettingFile = cameraYaml;
      printf("[depth-vslam] camera calibration from 'camera_yaml' param: %s\n", cameraSettingFile.c_str());
   }
   else
   {
      GetCameraSettingFile( sensorSetting, "Configuration/vslam.cfg", cameraSettingFile);
      printf("[depth-vslam] camera calibration from vslam.cfg Camera line: %s\n", cameraSettingFile.c_str());
   }

   // Absolute path -> use as-is; otherwise resolve relative to sensor_setting.
   std::string cameraCalibPath = ( !cameraSettingFile.empty() && cameraSettingFile[0] == '/' )
                                 ? cameraSettingFile : sensorSetting + cameraSettingFile;
   std::shared_ptr<CameraInterface> inputCamera = std::make_shared<InputRGBDCameraROS2>(g_node, cameraCalibPath);
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
   point_cloud_pub = nullptr;
   g_node = nullptr;
   printf("release ros node done\n");
   return 0;
}
