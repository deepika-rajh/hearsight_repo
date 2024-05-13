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
#include <iostream>

#include "SystemTime.h"
#include "VSLAMIMU.h"
#include "VSLAMSystem.h"

#include "sensor_client.h"

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

void VSLAMIMU::imuProc()
{
   SensorClient sensor_client;
   bool ret = false;
   int try_num = 0;

   do {
      ret = sensor_client.CreateConnection();
      try_num ++;
      std::this_thread::sleep_for(std::chrono::microseconds(2000));
   }while( !ret && try_num < 3);

   if (!ret) {
      std::cout << "IMU sensor_client: CreateConnection failed" << std::endl;
      return;
   }

   int32_t pack_num = 0;
   sensors_event_t* accel_ptr;
   sensors_event_t* gyro_ptr;
   float accVal[3], gyrVal[3];
   float delta = 0.f;
   int64_t lastTimeStamp = 0;

   while( threadRunning )
   {
      pack_num = 0;
      if (!sensor_client.GetImuData(&accel_ptr, &gyro_ptr, &pack_num)) {
         std::this_thread::sleep_for(std::chrono::microseconds(2000));
         continue;
      }

      //recorder.Record( accel_ptr, gyro_ptr, pack_num );

      for( int j = 0; j < pack_num; j++ )
      {
         int64_t curTimeStampNs = accel_ptr->timestamp;
         if( lastTimeStamp != 0 )
         {
            delta = (curTimeStampNs - lastTimeStamp)*1e-6f;
            if( delta > 50.0 )
               printf( "SensorInterarrival > 50ms :%fms \n", delta );
         }
         lastTimeStamp = curTimeStampNs;
         //Assume that we have both accel and gyro every sample.
         accVal[0] = accel_ptr->acceleration.x;
         accVal[1] = accel_ptr->acceleration.y;
         accVal[2] = accel_ptr->acceleration.z;
         gyrVal[0] = gyro_ptr->gyro.x;
         gyrVal[1] = gyro_ptr->gyro.y;
         gyrVal[2] = gyro_ptr->gyro.z;

         publishImuRaw( curTimeStampNs, gyrVal[0], gyrVal[1], gyrVal[2], accVal[0], accVal[1], accVal[2] );

         accel_ptr += 1;
         gyro_ptr += 1;
      }

      VSLAM_SLEEP( 20 );
   }

   sensor_client.DisconnectServer();
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
