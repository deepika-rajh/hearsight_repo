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

#include "VWSLAM.h"
#include "VSLAMSystem.h"

//static members definition
bool VSLAMSystem::showImg = false;
std::shared_ptr<VSLAMSystem> VSLAMSystem::t = nullptr;
std::shared_ptr<VSLAMIMU> VSLAMSystem::imu = nullptr;
std::shared_ptr<VSLAMWheel> VSLAMSystem::wheel = nullptr;
std::shared_ptr<VSLAMHijack> VSLAMSystem::hijack = nullptr;
std::shared_ptr<VWSLAM> VSLAMSystem::vwSLAM = nullptr;
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

extern rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr raw_pose_pub;

void pub_camera_raw_pose(const VWSLAM::VWSLAMPose & pose)
{
  auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();

  odom_msg->header.frame_id = "odom";
  odom_msg->child_frame_id  = "base_link";
  odom_msg->header.stamp = rclcpp::Time(pose.timestamp, RCL_ROS_TIME);

  odom_msg->pose.pose.position.x = pose.pose.position.x;
  odom_msg->pose.pose.position.y = pose.pose.position.y;
  odom_msg->pose.pose.position.z = pose.pose.position.z;

  double q[4];
  Euler2Quaternion(pose.pose.euler.roll, pose.pose.euler.pitch, pose.pose.euler.yaw, q);

  odom_msg->pose.pose.orientation.x = q[1];
  odom_msg->pose.pose.orientation.y = q[2];
  odom_msg->pose.pose.orientation.z = q[3];
  odom_msg->pose.pose.orientation.w = q[0];

  odom_msg->twist.twist.linear.x  = 0;
  odom_msg->twist.twist.angular.z = 0;

  raw_pose_pub->publish(std::move(odom_msg));
}
#endif
/**********************   C APIs end   ************************************/


VSLAMSystem::~VSLAMSystem()
{
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
}

std::shared_ptr<VSLAMSystem> VSLAMSystem::Initialize( const std::string & root, const std::string & outputDir, bool _showImg)
{
   rootPath = root;
   outputPath = outputDir;
    if (t.get() == nullptr) {
        t = std::make_shared<VSLAMSystem>();
        
#ifdef WHEEL_SUPPORTED
       wheel = std::make_shared<VSLAMWheel>( );
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

       vwSLAM = getVWSLAM();
       vwSLAM->init( rootPath, outputPath, configuration);

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

    //VSLAMSystem::doMapping = mapping;

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

   vwSLAM->run();
   printf("vwSLAM OK\n");
   systemState = KWORKING;
}

void VSLAMSystem::sleep(bool isCloseCamera)
{
#ifdef ARM_BASED
   if (isCloseCamera )
      inputCamera->stop();
#endif
   systemState = KSLEEPING;
   vwSLAM->sleep();
}

void VSLAMSystem::awake()
{
#ifdef ARM_BASED
   inputCamera->start();
#endif
   systemState = KWORKING;
   vwSLAM->awake();
}

void VSLAMSystem::reset()
{
   vwSLAM->stop();
   vwSLAM->reset();
   vwSLAM->run();
   if( systemState == KSLEEPING )
   {
      vwSLAM->awake();
   }
}


void VSLAMSystem::addImageToVslam( const int64_t timestamp, const uint8_t * imageBuf, const uint16_t * depthBuf )
{
   if( VSLAMSystem::systemState == KSLEEPING)
   {
      return;
   }
   printf("got an image\n");
   VWSLAM::VWSLAMStatus status;
   VWSLAM::VWSLAMPose rawPose;
   vwSLAM->addImage(timestamp, imageBuf, depthBuf);
   vwSLAM->getUndistortedImage( viz->getUndistortedImageBuf(), viz->getImageWidth(), viz->getImageHeight() );
   vwSLAM->getVWSLAMStatus( status ); 
   rawPose = vwSLAM->getVSLAMRawPose();
#ifdef ROS_BASED
   if(rawPose.poseQuality >= VWSLAM::QUALITY_GREAT )
   {
       pub_camera_raw_pose(rawPose);
   }
#endif

   viz->ShowPoints( rawPose.poseQuality, "points", status );

   if( VSLAMSystem::isMappingEnabled() )
   {
      vwSLAM->getGridImage( viz->getVisData(), viz->getVisWidth(), viz->getVisHeight() );
      viz->ShowGridMap();
   }

   if (status.observationBuf )
      delete status.observationBuf;
}

void
VSLAMSystem::addWheelOdom( float linearVelocity, float angularVelocity,
                   const float location[3], const float direction[4], int64_t timestamp )
{

   //printf("got an wheel\n");
   vwSLAM->addWheelOdom( linearVelocity, angularVelocity, location, direction, timestamp );
}

void
VSLAMSystem::addIMU( const float linearAcceleration[3], const float angularVelocity[3], int64_t timestamp )
{
   
   //printf("got an imu\n");
   vwSLAM->addImu( linearAcceleration, angularVelocity, timestamp );
}

void 
VSLAMSystem::addHijack( bool status, int64_t timestamp )
{

   //printf("got a hijack\n");
   vwSLAM->addHijack( status, timestamp );
}


void 
VSLAMSystem::Spin()
{
#ifdef ROS_BASED
   extern rclcpp::Node::SharedPtr g_node;
   rclcpp::spin( g_node );
#else
   while( systemState != KSTOPPING )
   {
      VSLAM_SLEEP( 1 );

   }

#endif
}

