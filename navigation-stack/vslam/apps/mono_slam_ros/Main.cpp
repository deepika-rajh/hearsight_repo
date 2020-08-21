/*****************************************************************************
@copyright
Copyright (c) 2020 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include <stdlib.h>
#include <iostream>
#include <string>
#include <unistd.h>

#include "VSLAMSystem.h"
#include "InputWheelROS.h"

//ROS2 common headers
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <image_transport/image_transport.h>
#include <nav_msgs/msg/odometry.hpp>

bool debugLevel = 0;
static char *helpMsg =
      "mv_vwslam \n"
      "Usage: mv_vwslam [-options]\n"
      "-c : set configuration files path, default path is /data/misc/vwslam/ \n"
      "-o : set output files path, default path is /data/vwslam/ \n"
      "-d : set vslam debug level: enable debug info(1), disable debug info(0) \n"
      "-v : get vslam app version \n"
      "-h : print help msg\n";

rclcpp::Node::SharedPtr g_node = nullptr;
image_transport::Publisher    color_pub;
image_transport::Publisher    labeled_img_pub;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr raw_pose_pub = nullptr;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr robot_pose_pub = nullptr;

int main( int argc, char** argv )
{
   int opt;

   std::string root = std::string( "/data/misc/vwslam/" );
   std::string output = std::string( "/data/vwslam/" );

   if( argc < 2 )
   {
      printf( "%s run with default setting.\n", argv[0] );
   }
   else
   {
      while((opt = getopt(argc, argv, "c:o:i:d:vh")) != -1)
      {
        switch(opt) {
         case 'c':
            root = std::string(optarg);
          break;

         case 'o':
            output = std::string(optarg);
          break;

         case 'd':
            debugLevel = atoi(optarg);
            printf("VSLAM debug level is %d\n", debugLevel);
          break;

         case 'v':
            printf( "%s version: %s \n", argv[0], VSLAM_APP_VERSION);
            return 0;

        case 'h':
        default:
            printf("%s", helpMsg);
            return 1;
         }
      }
   }
   
   rclcpp::init(argc, argv);
   g_node = rclcpp::Node::make_shared("rv_vwslam_ros");
   
   color_pub = image_transport::create_publisher(g_node.get(), "camera/gray_image");
   labeled_img_pub = image_transport::create_publisher(g_node.get(), "vslam/labeled_img");
   raw_pose_pub = g_node.get()->create_publisher<nav_msgs::msg::Odometry>("vslam_odom_raw", 5);
   robot_pose_pub = g_node.get()->create_publisher<nav_msgs::msg::Odometry>("robot_odom", 5);

   char tmp = *(output.end() - 1);
   if( tmp != '/' && tmp != '\\' )
   {
      output = output + '/';
   }

   tmp = *(root.end() - 1);
   if( tmp != '/' && tmp != '\\' )
   {
      root = root + '/';
   }

#ifdef ARM_BASED
   //add log for ARM platform to check boot time
   system("echo vSLAM Start Initialization > /dev/kmsg");
#endif

   InputWheelROS wheel;

   //start VSLAM system
   std::shared_ptr<VSLAMSystem> sys = VSLAMSystem::Initialize(root, output, false, false);
   sys->Run();

   //wait to quit
   sys->Spin();

   //stop VSLAM
   sys->Quit();
   sys->deinit();
   sys->state_sub = nullptr;
   sys = nullptr;
   printf("vslam application exits\n");
   fflush(stdout);

   color_pub.shutdown();
   labeled_img_pub.shutdown();
   rclcpp::shutdown();
   raw_pose_pub = nullptr;
   robot_pose_pub = nullptr;
   g_node = nullptr;
   printf("release ros node done\n");
   return 0;
} 
