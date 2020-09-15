/*****************************************************************************
@copyright
Copyright (c) 2020 Qualcomm Technologies, Inc.
All Rights Reserved.
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

#include "VSLAMSystem.h"
#include "rvVWSLAM.h"
#include "SystemTime.h"

//static members definition
bool VSLAMSystem::showImg = false;
//std::shared_ptr<rvVWSLAM> VSLAMSystem::vslamPtr = nullptr;
rvVWSLAM *VSLAMSystem::vslamPtr = nullptr;
std::shared_ptr<VSLAMSystem> VSLAMSystem::t = nullptr;
std::shared_ptr<VSLAMIMU> VSLAMSystem::imu = nullptr;
std::shared_ptr<VSLAMWheel> VSLAMSystem::wheel = nullptr;
std::shared_ptr<VSLAMHijack> VSLAMSystem::hijack = nullptr;
//std::shared_ptr<rvVWSLAM> VSLAMSystem::vwSLAM = nullptr;
VSLAMSystem::SystemState VSLAMSystem::systemState = KSLEEPING;
std::shared_ptr<Visualiser> VSLAMSystem::viz = nullptr;
std::string VSLAMSystem::rootPath = "";
std::string VSLAMSystem::outputPath = "";
rvCameraParams VSLAMSystem::configuration;
bool VSLAMSystem::doMapping = false;
#define PLAYBACK_CONFIGURATION "Configuration/vslam.cfg"
#ifdef ARM_BASED
#ifdef ENABLE_DEPTH
std::shared_ptr<InputCamera_D435i> VSLAMSystem::inputCamera = nullptr;
#else
std::shared_ptr<InputCamera_OV9282> VSLAMSystem::inputCamera = nullptr;
#endif
#else
std::shared_ptr<camera::VirtualSensorDevice> VSLAMSystem::inputCamera = nullptr;
#endif


/************************************** C APIs start ************/
void Euler2Quaternion( double roll, double pitch, double yaw, double quaternion[4] )
{
   double t0 = cos( yaw * 0.5 );
   double t1 = sin( yaw * 0.5 );
   double t2 = cos( roll * 0.5 );
   double t3 = sin( roll * 0.5 );
   double t4 = cos( pitch * 0.5 );
   double t5 = sin( pitch * 0.5 );

   quaternion[0] = t0 * t2 * t4 + t1 * t3 * t5;  //w
   quaternion[1] = t0 * t3 * t4 - t1 * t2 * t5;  //x
   quaternion[2] = t0 * t2 * t5 + t1 * t3 * t4;  //y
   quaternion[3] = t1 * t2 * t4 - t0 * t3 * t5;  //z
}

#ifdef ROS_BASED
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
using std::placeholders::_1;

extern rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr raw_pose_pub;
extern rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr robot_pose_pub;
extern rclcpp::Node::SharedPtr g_node;

void pub_camera_raw_pose(const rvVSLAMPose & pose)
{
  auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();

  odom_msg->header.frame_id = "odom";
  odom_msg->child_frame_id  = "base_link";
  odom_msg->header.stamp = rclcpp::Time(pose.timestampNs, RCL_ROS_TIME);

  odom_msg->pose.pose.position.x = pose.pose.translation[0];
  odom_msg->pose.pose.position.y = pose.pose.translation[1];
  odom_msg->pose.pose.position.z = pose.pose.translation[2];

  double q[4];
  Euler2Quaternion(pose.pose.euler[0], pose.pose.euler[1], pose.pose.euler[2], q);

  odom_msg->pose.pose.orientation.x = q[1];
  odom_msg->pose.pose.orientation.y = q[2];
  odom_msg->pose.pose.orientation.z = q[3];
  odom_msg->pose.pose.orientation.w = q[0];

  odom_msg->twist.twist.linear.x  = 0;
  odom_msg->twist.twist.angular.z = 0;

  raw_pose_pub->publish(std::move(odom_msg));
}

void pub_robot_pose(const rvVSLAMPose & pose)
{
  auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();

  odom_msg->header.frame_id = "odom";
  odom_msg->child_frame_id  = "base_link";
  odom_msg->header.stamp = rclcpp::Time(pose.timestampNs, RCL_ROS_TIME);

  odom_msg->pose.pose.position.x = pose.pose.translation[0];
  odom_msg->pose.pose.position.y = pose.pose.translation[1];
  odom_msg->pose.pose.position.z = pose.pose.translation[2];

  double q[4];
  Euler2Quaternion(pose.pose.euler[0], pose.pose.euler[1], pose.pose.euler[2], q);

  odom_msg->pose.pose.orientation.x = q[1];
  odom_msg->pose.pose.orientation.y = q[2];
  odom_msg->pose.pose.orientation.z = q[3];
  odom_msg->pose.pose.orientation.w = q[0];

  odom_msg->twist.twist.linear.x  = 0;
  odom_msg->twist.twist.angular.z = 0;

  robot_pose_pub->publish(std::move(odom_msg));
}

void VSLAMSystem::state_callback(const std_msgs::msg::String::SharedPtr msg) const
{
  printf("vslam received state change msg: %s\n", msg->data.c_str());

  if (!strcmp(msg->data.c_str(), "stop"))
  {
      printf("vslam received stop sig\n");
      Stop(SIGINT);
  }else if (!strcmp(msg->data.c_str(), "sleep" )) {
      printf("vslam received sleep sig\n");
      sleep(false);
  }else if (!strcmp(msg->data.c_str(), "awake" )) {
      printf("vslam received awake sig\n");
      awake();
  }else if (!strcmp(msg->data.c_str(), "reset" )) {
      printf("vslam received reset sig\n");
      reset();
  }

}
#endif
/**********************   C APIs end   ************************************/

VSLAMSystem::~VSLAMSystem()
{
}

void VSLAMSystem::deinit()
{
   rvVWSLAM_Deinitialize( vslamPtr );

   inputCamera = nullptr;
   wheel = nullptr;
   hijack = nullptr;
   imu = nullptr;
   vslamPtr = nullptr;
}

VSLAMSystem::VSLAMSystem()
{
   systemState = KSLEEPING;
#ifdef ARM_BASED
#ifdef ENABLE_DEPTH
   inputCamera = std::make_shared<InputCamera_D435i>();
#else
   inputCamera = std::make_shared<InputCamera_OV9282>( PLAYBACK_CONFIGURATION );
#endif

   if( inputCamera )
   {
      inputCamera->addCallback( addImageToVslam );
   }

#else
   inputCamera = std::make_shared<camera::VirtualSensorDevice>( PLAYBACK_CONFIGURATION );
#endif

   if( inputCamera )
   {
      inputCamera->start();
      configuration = inputCamera->getCameraConfiguration();
      //inputCamera->stop();
   }

#ifdef ROS_BASED
   state_sub = g_node->create_subscription<std_msgs::msg::String>( "vslam_state", 10,
        std::bind(&VSLAMSystem::state_callback, this,  _1));
#endif
}

std::shared_ptr<VSLAMSystem> VSLAMSystem::Initialize( const std::string & root, const std::string & outputDir, bool _showImg, const bool mapping )
{
   rootPath = root;
   outputPath = outputDir;

   if( t.get() == nullptr )
   {
      t = std::make_shared<VSLAMSystem>();

#ifdef WHEEL_SUPPORTED
      wheel = std::make_shared<VSLAMWheel>();
      if( wheel )
      {
         std::shared_ptr<WheelOdomReceiver> tmp = t;
         wheel->addReceiver( tmp );
      }
#endif
      hijack = std::make_shared<VSLAMHijack>();
      if( hijack )
      {
         std::shared_ptr<HijackReceiver> tmp = t;
         hijack->addReceiver( tmp );
      }

      vslamPtr = rvVWSLAM_Initialize( root.c_str(), outputPath.c_str(), &configuration, mapping );

#ifdef IMU_SUPPORTED
      int32_t imuAxleSign[3] = { 1, 1, -1 }; //need to be read from configuration file in future
      imu = std::make_shared<VSLAMIMU>( imuAxleSign );
      if( imu != nullptr )
      {
         std::shared_ptr<IMUReceiver> tmp = t;
         imu->addReceiver( tmp );
      }
#endif
      showImg = _showImg;

      viz = std::make_shared<Visualiser>( configuration.outputPixelWidth, configuration.outputPixelHeight );

#if GDB_DEBUG  //SIGINT would go to gdb but not vslam application
      signal( 48, Stop );
#else
      signal( SIGINT, Stop );
#endif
   }

   VSLAMSystem::doMapping = mapping;

   return t;
}

void VSLAMSystem::Run()
{
   if( systemState == KSTOPPING )
      return;

   if( imu )
      imu->start();
   if( wheel )
      wheel->start();
   if( hijack )
      hijack->start();
   //wod->start();


   //if( inputCamera )
   //   inputCamera->start();

   //const rvCameraParams & configuration = inputCamera->getCameraConfiguration();

   //    vwSLAM = getVWSLAM();
   //    vwSLAM->init( rootPath, outputPath, configuration );

   //vwSLAM->run();
   rvVWSLAM_Run(vslamPtr);

   printf("vwSLAM OK\n");
   systemState = KWORKING;
}

void VSLAMSystem::sleep(bool isCloseCamera)
{
   if(systemState == KSLEEPING)
      return;

#ifdef ARM_BASED
   if (isCloseCamera )
      inputCamera->stop();
#endif
   systemState = KSLEEPING;
   rvVWSLAM_Sleep(vslamPtr);
}

void VSLAMSystem::awake(bool isStartCamera)
{
   if(systemState == KSLEEPING)
   {
#ifdef ARM_BASED
   if(isStartCamera)
      inputCamera->start();
#endif
   systemState = KWORKING;
   rvVWSLAM_Awake(vslamPtr);
   }
}

void VSLAMSystem::reset()
{
   rvVWSLAM_Stop(vslamPtr);
   rvVWSLAM_Reset(vslamPtr);
   rvVWSLAM_Run(vslamPtr);
   if( systemState == KSLEEPING )
   {
      systemState = KWORKING;
      rvVWSLAM_Awake(vslamPtr);
   }
}

static uint64_t lastPoseTimeStamp = 0;
static int isInitDone = 0;
void VSLAMSystem::addImageToVslam( const int64_t timestamp, const uint8_t * imageBuf, const uint16_t * depthBuf )
{
   if( VSLAMSystem::systemState == KSLEEPING)
   {
      return;
   }

   //printf("got an image\n");
   if( !isInitDone )
   {
      system("echo vSLAM Initialization is finished > /dev/kmsg");
      isInitDone = true;
   }

   rvVWSLAMStatus status;
   rvVSLAMPose rawPose, robotPose;

   rvVWSLAM_AddImage(vslamPtr, timestamp, imageBuf, depthBuf);
   rawPose = rvVWSLAM_GetVslamRawPose(vslamPtr);
#ifdef ROS_BASED
   if(rawPose.poseQuality >= RV_VSLAM_TRACKING_STATE_GREAT )
   {
       pub_camera_raw_pose(rawPose);
   }
#endif

   robotPose = rvVWSLAM_GetVslamOutputPose(vslamPtr);
   int64_t current_time = (int64_t)getRealTime();

   if( robotPose.timestampNs - lastPoseTimeStamp > 0)
   {
      lastPoseTimeStamp = robotPose.timestampNs;
      printf("pose is updated. cur_time %ld, last %ld, the latency is %ld ms\n", current_time, robotPose.timestampNs, (current_time - robotPose.timestampNs)/1000000);
      #ifdef ROS_BASED
      pub_robot_pose(robotPose);
      #endif
   }

#ifndef WIN32
   rvVWSLAM_getUndistortedImage(vslamPtr, viz->getUndistortedImageBuf(), viz->getImageWidth(), viz->getImageHeight() );
   rvVWSLAM_GetVWSLAMStatus(vslamPtr, &status);
   viz->ShowPoints( rawPose.poseQuality, "points", status );
#endif

   if( VSLAMSystem::isMappingEnabled() )
   {
	   rvVWSLAM_getGridImage(vslamPtr, viz->getVisData(), viz->getVisWidthAddr(), viz->getVisHeightAddr() );
	   viz->ShowGridMap();
   }

   if (status.observationBuf )
      delete status.observationBuf;
}

void
VSLAMSystem::addWheelOdom( float linearVelocity, float angularVelocity,
                   const float location[3], const float direction[4], int64_t timestamp )
{
	rvVWSLAM_AddWheelOdom(vslamPtr, linearVelocity, angularVelocity, location, direction, timestamp );
}

void
VSLAMSystem::addIMU( const float linearAcceleration[3], const float angularVelocity[3], int64_t timestamp )
{
   rvVWSLAM_AddImu(vslamPtr, linearAcceleration, angularVelocity, timestamp );
}

void 
VSLAMSystem::addHijack( bool status, int64_t timestamp )
{
	rvVWSLAM_AddHijack(vslamPtr, status, timestamp );
}


void 
VSLAMSystem::Spin()
{
#ifdef ROS_BASED
   rclcpp::spin( g_node );
#else
   while( systemState != KSTOPPING )
   {
      VSLAM_SLEEP( 1 );

   }

#endif
}

