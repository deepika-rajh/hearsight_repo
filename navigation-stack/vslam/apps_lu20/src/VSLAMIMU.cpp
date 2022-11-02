/*****************************************************************************
@copyright
Copyright (c) 2020-2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <functional>

#include "SystemTime.h"
#include "VSLAMIMU.h"
#include "VSLAMSystem.h"

#include "imu_client.hpp"
#define SAMPLE_RATE  200

#ifdef ROS_BASED
#include <sensor_msgs/msg/imu.hpp>
extern rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;

void publishImuRaw( int64_t ts, float gry_x, float gry_y, float gry_z, float acc_x, float acc_y, float acc_z )
{
   rclcpp::Time t = rclcpp::Time( ts, RCL_ROS_TIME );
   sensor_msgs::msg::Imu imu_msg;

   imu_msg.header.stamp = t;
   imu_msg.header.frame_id = "imu";

   imu_msg.linear_acceleration.x = acc_x;
   imu_msg.linear_acceleration.y = acc_y;
   imu_msg.linear_acceleration.z = acc_z;
   imu_msg.angular_velocity.x = gry_x;
   imu_msg.angular_velocity.y = gry_y;
   imu_msg.angular_velocity.z = gry_z;
   imu_pub->publish( imu_msg );
}
#elif ROS1_BASED
#include <sensor_msgs/Imu.h>
extern ros::Publisher imu_pub;

void publishImuRaw( int64_t ts, float gry_x, float gry_y, float gry_z, float acc_x, float acc_y, float acc_z )
{
   //rclcpp::Time t = rclcpp::Time(ts, RCL_ROS_TIME);
   sensor_msgs::Imu imu_msg;

   imu_msg.header.stamp = ros::Time::now();
   imu_msg.header.frame_id = "imu";

   imu_msg.linear_acceleration.x = acc_x;
   imu_msg.linear_acceleration.y = acc_y;
   imu_msg.linear_acceleration.z = acc_z;
   imu_msg.angular_velocity.x = gry_x;
   imu_msg.angular_velocity.y = gry_y;
   imu_msg.angular_velocity.z = gry_z;
   imu_pub.publish( imu_msg );
}
#endif

void VSLAMIMU::imuProc()
{
   ImuClient _imu_client;
   struct imu_pack_dsp imu_data[64];
   int32_t pack_num = 0;
   int rate = SAMPLE_RATE;

   if( !_imu_client.InitMmap() )
   {
      printf( "imu: init mmap failed" );
      return;
   }

   if( !_imu_client.ConnectServer() )
   {
      printf( "imu: connect imud failed" );
      return;
   }

   if( !_imu_client.SendMsgConfigDataType( ACCEL_TYPE ) )
   {
      printf( "imu: send acc CONFIG_TYPE to imud failed" );
      return;
   }

   if( !_imu_client.SendMsgConfigRate( rate ) )
   {
      printf( "imu: send acc CONFIG_RATE to imud failed" );
      return;
   }

   if( !_imu_client.SendMsgConfigDataType( ACCEL_TYPE ) )
   {
      printf( "imu: send gyro CONFIG_TYPE to imud failed" );
      return;
   }

   if( !_imu_client.SendMsgConfigRate( rate ) )
   {
      printf( "imu: send gyro CONFIG_RATE to imud failed" );
      return;
   }

   if( !_imu_client.SendMsgStart( ACCEL_TYPE ) )
   {
      printf( "send START accel to imud failed" );
      return;
   }

   if( !_imu_client.SendMsgStart( GYRO_TYPE ) )
   {
      printf( "send START gyro to imud failed" );
      return;
   }

   float accVal[3], gyrVal[3];
   float delta = 0.f;
   int64_t lastTimeStamp = 0;

   while( threadRunning )
   {
      _imu_client.GetImuData( imu_data, 64, &pack_num );

      recorder.Record( imu_data, clockOffset, pack_num );

      for( int j = 0; j < pack_num; ++j )
      {
         int64_t curTimeStampNs = imu_data[j].time_acc + clockOffset;
         
         if( lastTimeStamp != 0 )
         {
            delta = (curTimeStampNs - lastTimeStamp)*1e-6f;
            if( delta > 50.0 )
               printf( "SensorInterarrival > 50ms :%fms \n", delta );
         }
         lastTimeStamp = curTimeStampNs;
         //Assume that we have both accel and gyro every sample.
         accVal[0] = imu_data[j].acceloration_x;
         accVal[1] = imu_data[j].acceloration_y;
         accVal[2] = imu_data[j].acceloration_z;
         gyrVal[0] = imu_data[j].angular_velocity_x;
         gyrVal[1] = imu_data[j].angular_velocity_y;
         gyrVal[2] = imu_data[j].angular_velocity_z;

         for( size_t i = 0; i < receiverVector.size(); i++ )
         {
            receiverVector[i]->addIMU( accVal, gyrVal, curTimeStampNs );
         }

#ifdef ROS_BASED
         publishImuRaw( curTimeStampNs, gyrVal[0], gyrVal[1], gyrVal[2], accVal[0], accVal[1], accVal[2] );
#endif

      }

      VSLAM_SLEEP( 20 );
   }
}

VSLAMIMU::VSLAMIMU( ): recorder("/data/vwslam")
{
   threadRunning = false;

   realClock = getRealTime();
   monotonicClock = getMonotonicTime();
   clockOffset = realClock - monotonicClock;
}


VSLAMIMU::~VSLAMIMU()
{}

bool VSLAMIMU::init()
{
   return true;
}

void VSLAMIMU::stop()
{
   threadRunning = false;
   if( imuPollThread )
   {
      imuPollThread->join();
   }
}

void VSLAMIMU::start()
{
   threadRunning = true;
   init();

   imuPollThread = std::make_shared<std::thread>( std::mem_fn( &VSLAMIMU::imuProc ), this );
}
