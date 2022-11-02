/*****************************************************************************
@copyright
Copyright (c) 2020-2022 Qualcomm Technologies, Inc.
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

#include <sstream>
#include <fstream>

#include <rvQueue.h>
queue_mt<sensor_hijack> hijackArray(BUF_SIZE);

//static members definition
rvVWSLAM *VSLAMSystem::vslamPtr = nullptr;
std::shared_ptr<VSLAMSystem> VSLAMSystem::t = nullptr;
std::shared_ptr<VSLAMWheel> VSLAMSystem::wheel = nullptr;
VSLAMSystem::SystemState VSLAMSystem::systemState = KSLEEPING;
std::shared_ptr<Visualiser> VSLAMSystem::viz = nullptr;
std::string VSLAMSystem::algConfFile = "";
std::string VSLAMSystem::outputPath = "";
rvCameraParams VSLAMSystem::cameraConfiguration;
rvIMUConfiguration VSLAMSystem::imuConfiguration;
rvWheelConfiguration VSLAMSystem::wheelConfiguration;
rvTargetImage VSLAMSystem::targetImage;

#ifdef IMU_SUPPORTED
   std::shared_ptr<VSLAMIMU> VSLAMSystem::imu = nullptr;
#endif

#ifdef HIJACK_SUPPORTED
std::shared_ptr<VSLAMHijack> VSLAMSystem::hijack = nullptr;
#endif //HIJACK_SUPPROTED

std::shared_ptr<CameraInterface> VSLAMSystem::inputCamera = nullptr;


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

void VSLAMSystem::state_callbackROS(const std_msgs::msg::String::SharedPtr msg) const
{
    state_callback(msg->data);
}
#endif
/**********************   C APIs end   ************************************/

VSLAMSystem::~VSLAMSystem()
{
}

void VSLAMSystem::deinit()
{
   rvVWSLAM_SaveMap( vslamPtr, NULL, NULL );
   rvVWSLAM_Deinitialize( vslamPtr );

   inputCamera = nullptr;
   wheel = nullptr;

#ifdef HIJACK_SUPPORTED
   hijack = nullptr;
#endif //HIJACK_SUPPORTED

#ifdef IMU_SUPPORTED
   imu = nullptr;
#endif
   vslamPtr = nullptr;
}

VSLAMSystem::VSLAMSystem( std::shared_ptr<CameraInterface> & camera )
{
   systemState = KSLEEPING;

   inputCamera = camera;
   if( inputCamera )
   {
      inputCamera->addCallback(addImageToVslam);
      inputCamera->start();
      cameraConfiguration = inputCamera->getCameraConfiguration();
      //inputCamera->stop();
   }

#ifdef ROS_BASED
   state_sub = g_node->create_subscription<std_msgs::msg::String>( "vslam_state", 10,
        std::bind(&VSLAMSystem::state_callbackROS, this,  _1));
#endif
}

void EulerToSO3_1( const float32_t* euler, float32_t* rotation )
{
   float32_t cr = (float32_t)cos( euler[0] );
   float32_t sr = (float32_t)sin( euler[0] );
   float32_t cp = (float32_t)cos( euler[1] );
   float32_t sp = (float32_t)sin( euler[1] );
   float32_t cy = (float32_t)cos( euler[2] );
   float32_t sy = (float32_t)sin( euler[2] );
   rotation[0 * 3 + 0] = cy*cp;
   rotation[0 * 3 + 1] = cy*sp*sr - sy*cr;
   rotation[0 * 3 + 2] = cy*sp*cr + sy*sr;
   rotation[1 * 3 + 0] = sy*cp;
   rotation[1 * 3 + 1] = sy*sp*sr + cy*cr;
   rotation[1 * 3 + 2] = sy*sp*cr - cy*sr;
   rotation[2 * 3 + 0] = -sp;
   rotation[2 * 3 + 1] = cp*sr;
   rotation[2 * 3 + 2] = cp*cr;
}


std::shared_ptr<VSLAMSystem> VSLAMSystem::Initialize( const std::string & algSetting,  const std::string & outputDir,
                                                      std::shared_ptr<CameraInterface> camera, bool _showImg )
{
   algConfFile = algSetting;
   outputPath = outputDir;

   printf("***ZYM*** initialization started\n");
   if( t.get() == nullptr )
   {
      t = std::make_shared<VSLAMSystem>(camera);
#ifdef WHEEL_SUPPORTED
      wheel = std::make_shared<VSLAMWheel>();
      if( wheel )
      {
         std::shared_ptr<WheelOdomReceiver> tmp = t;
         wheel->addReceiver( tmp );
      }
#endif

#ifdef HIJACK_SUPPORTED
      hijack = std::make_shared<VSLAMHijack>();
      if( hijack )
      {
         std::shared_ptr<HijackReceiver> tmp = t;
         hijack->addReceiver( tmp );
      }
#endif //HIJACK_SUPPORTED

      vslamPtr = rvVWSLAM_Initialize( algConfFile.c_str(), outputPath.c_str(), 
                                      &cameraConfiguration, &wheelConfiguration, &imuConfiguration, & targetImage );

#ifdef IMU_SUPPORTED
      imu = std::make_shared<VSLAMIMU>();
      if( imu != nullptr )
      {
         std::shared_ptr<IMUReceiver> tmp = t;
         imu->addReceiver( tmp );
      }
#endif
      switch(cameraConfiguration.cameraType)
      {
      case rvStereo:
         viz = std::make_shared<Visualiser>( cameraConfiguration.stereo.camera[0].pixelWidth, cameraConfiguration.stereo.camera[0].pixelHeight );
         break;
      case rvGrayDepth:
         viz = std::make_shared<Visualiser>( cameraConfiguration.stereo.camera[0].pixelWidth, cameraConfiguration.stereo.camera[0].pixelHeight );
         break;
      case rvMonocular:
      default:
         viz = std::make_shared<Visualiser>( cameraConfiguration.stereoRect.camera[0].pixelWidth, cameraConfiguration.stereoRect.camera[0].pixelHeight );
      }

#if GDB_DEBUG  //SIGINT would go to gdb but not vslam application
      signal( 48, Stop );
#else
      signal( SIGINT, Stop );
#endif
   }

   return t;
}

void VSLAMSystem::Run()
{
   if( systemState == KSTOPPING )
   {
      return;
   }
#ifdef IMU_SUPPORTED
   if( imu )
      imu->start();
#endif
   if( wheel )
      wheel->start();

#ifdef HIJACK_SUPPORTED
   if( hijack )
      hijack->start();
#endif //HIJACK_SUPPORTED

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

#ifdef SIMULATION
std::mutex VSLAMSystem::mut;
uint64_t VSLAMSystem::currentImageTimeStamp = 1;
uint64_t VSLAMSystem::rawPoseTimeStamp = 0;
std::condition_variable VSLAMSystem::data_cond;
#endif

void VSLAMSystem::addImageToVslam( const int64_t timestamp, const uint8_t * imageBuf, const uint16_t * depthBuf )
{
   if( VSLAMSystem::systemState == KSLEEPING)
   {
      return;
   }

   printf("got an image\n");
   if( !isInitDone )
   {
      system("echo vSLAM Initialization is finished > /dev/kmsg");
      isInitDone = true;
   }
#ifdef SIMULATION   
   {
      std::lock_guard<std::mutex> lk( mut );
      currentImageTimeStamp = timestamp;
   }
#endif

   rvVSLAMPose robotPose;
   rvVWSLAM_AddImage(vslamPtr, timestamp, imageBuf, depthBuf);

   robotPose = rvVWSLAM_GetBaselinkPose(vslamPtr);
   int64_t current_time = (int64_t)getRealTime();

   if( robotPose.timestampNs - lastPoseTimeStamp > 0)
   {
      lastPoseTimeStamp = robotPose.timestampNs;
      printf("pose is updated. cur_time %" PRId64 ", last %" PRId64 " the latency is %f ms\n", current_time, robotPose.timestampNs, (current_time - robotPose.timestampNs)/1000000.f);
      #ifdef ROS_BASED
      pub_robot_pose(robotPose);
      #endif
   }

#ifndef WIN32
   rvVWSLAMStatus status;
   rvVWSLAM_GetUndistortedImage( vslamPtr, viz->getUndistortedImageBuf(), viz->getImageWidth(), viz->getImageHeight() );
   int brightness = 0;
   for( int i = 0, pixelIn = 0; i < viz->getImageHeight(); i++ )
      for( int j = 0; j < viz->getImageWidth(); j++, pixelIn++ )
         brightness += viz->getUndistortedImageBuf()[pixelIn];
   status._Brightness = brightness / (viz->getImageWidth()* viz->getImageHeight());
   status._KeyframeNum = rvVWSLAM_GetKeyframeNumber( vslamPtr );
   status._ObservationBuf = NULL;
   status._MatchedMapPointNum = status._MisMatchedMapPointNum = 0;
   int obsNum = rvVWSLAM_GetVWSLAMObservations( vslamPtr, status._ObservationBuf, 0 );
   if( obsNum > 0 )
   {
      status._ObservationBuf = new RV_TrackedObservation[obsNum];
      obsNum = rvVWSLAM_GetVWSLAMObservations( vslamPtr, status._ObservationBuf, obsNum );
      for( int i = 0; i < obsNum; i++ )
      {
         if( status._ObservationBuf[i].s == RV_TrackedObservation::MATCHING_OK )
         {
            status._MatchedMapPointNum++;
         }
         else
         {
            status._MisMatchedMapPointNum++;
         }
      }
   }
   rvVSLAMPose rawPose = rvVWSLAM_GetVslamRawPose(vslamPtr);
   
#ifdef ROS_BASED
   if(rawPose.poseQuality >= RV_VSLAM_TRACKING_STATE_GREAT )
   {
       pub_camera_raw_pose(rawPose);
   }
#endif
   viz->ShowPoints( rawPose.poseQuality, "points", status );
   if( status._ObservationBuf )
      delete status._ObservationBuf;
#endif
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
#elif defined (ROS1_BASED)
   ros::spin( );
#else 

   // FILE * fp = fopen( "predict.csv", "wt" );
   while( systemState != KSTOPPING )
   {
       VSLAM_SLEEP( 10 );
#ifdef SIMULATION
       rvVSLAMPose  rawPose = rvVWSLAM_GetVslamRawPose(vslamPtr);
       {
          std::lock_guard<std::mutex> lk( mut );
          rawPoseTimeStamp = rawPose.timestampNs;
          data_cond.notify_all();
       }
#endif //SIMULATION
   }
#ifdef SIMULATION
   {
      std::lock_guard<std::mutex> lk( mut );
      rawPoseTimeStamp = currentImageTimeStamp;
      data_cond.notify_all();
   }
#endif //SIMULATION
#endif
}

void VSLAMSystem::state_callback(const std::string & msg)
{
    printf("vslam received state change msg: %s\n", msg.c_str());

    if (!strcmp(msg.c_str(), "stop"))
    {
        printf("vslam received stop sig\n");
        Stop(SIGINT);
    }
    else if (!strcmp(msg.c_str(), "sleep")) {
        printf("vslam received sleep sig\n");
        sleep(false);
    }
    else if (!strcmp(msg.c_str(), "awake")) {
        printf("vslam received awake sig\n");
        awake();
    }
    else if (!strcmp(msg.c_str(), "reset")) {
        printf("vslam received reset sig\n");
        reset();
    }
}

void VSLAMSystem::Quit( void )
{
   rvVWSLAM_Stop( vslamPtr );

#ifdef IMU_SUPPORTED
   if( imu )
      imu->stop();
#endif
   if( wheel )
      wheel->stop();

#ifdef HIJACK_SUPPORTED
   if( hijack )
      hijack->stop();
#endif //HIJACK_SUPPORTED

   if( inputCamera )
      inputCamera->stop();
}

void VSLAMSystem::waitForRawPose()
{
#ifdef SIMULATION
   std::unique_lock<std::mutex> lk( mut );
   data_cond.wait( lk, []
   {
      return rawPoseTimeStamp >= currentImageTimeStamp;
   } );
#endif //SIMULATION
}
