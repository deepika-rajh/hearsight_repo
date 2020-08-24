/*****************************************************************************
@copyright
Copyright (c) 2020 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/
#include "VSLAMWheel.h"
#include "wheel_datatype.h"
#include "string.h"
#include "VSLAMSystem.h"
#include <functional>
#include "rvQueue.h"

#define BUF_SIZE 10
queue_mt<sensor_wheel> wheelDataArray( BUF_SIZE );

void VSLAMWheel::wheelProc()
{
   int numSamples = 0;
   float delta = 0.f;
   sensor_wheel * sensorDataPtr = new sensor_wheel[BUF_SIZE];
   int64_t lastTimeStamp = 0;

   while( running )
   {
      getData( sensorDataPtr, 1, &numSamples );
      if( 0 )//numSamples )
      {
         printf( "numSamples is %d\n", numSamples );
      }
      for( int j = 0; j < numSamples; ++j )
      {
         sensor_wheel * curData = (sensor_wheel *)sensorDataPtr + j;
         int64_t curTimeStampNs = (int64_t)curData->timestamp;
         if( lastTimeStamp != 0 )
         {
            delta = (curTimeStampNs - lastTimeStamp)*1e-6f;
         }
         lastTimeStamp = curTimeStampNs;

         for( std::shared_ptr<WheelOdomReceiver> receiver : receivers )
         {
            receiver->addWheelOdom( curData->linear_velocity,
                                          curData->angular_velocity, 
                                          curData->location, 
                                          curData->direction, curTimeStampNs );
         }
      }

      // Sleep to avoid the crash caused by destroying the mutex while its busy
      // Would not a good idea and just a workaround
      //VSLAM_MASTER_SLEEP( 20 );
   }
   delete[]( sensor_wheel * )sensorDataPtr;

   printf("6. ----------> wheelProc thread exit \n");
}

VSLAMWheel::VSLAMWheel()
{
   running = false;
}

VSLAMWheel::~VSLAMWheel()
{
}


void VSLAMWheel::stop()
{
	running = false;
   sensor_wheel wheelodom;
   memset(&wheelodom, 0, sizeof(wheelodom));
   wheelDataArray.check_push(wheelodom);
   if( wheelPullThread )
   {
      wheelPullThread->join();
   }
}

void VSLAMWheel::start()
{
   running = true;
   wheelPullThread = std::make_shared<std::thread>(std::mem_fn(&VSLAMWheel::wheelProc), this);
}

int16_t VSLAMWheel::getData( sensor_wheel* dataArray, int32_t max_count, int32_t* available_data_num )
{
   int32_t i;
   for( i = 0; i < max_count; i++ )
   {
      sensor_wheel wheel;
      bool result = wheelDataArray.try_pop( wheel );
      if( !result )
         wheelDataArray.wait_and_pop( wheel );

      //{
         dataArray[i].timestamp = wheel.timestamp;
         dataArray[i].angular_velocity = wheel.angular_velocity;
         dataArray[i].linear_velocity = wheel.linear_velocity;
         memcpy( dataArray[i].location, wheel.location, sizeof( wheel.location ) );
         memcpy( dataArray[i].direction, wheel.direction, sizeof( wheel.direction ) );
      //}
      //else
      //{
      //   break;
      //}
   }
   *available_data_num = i;
   return 1;
}

void VSLAMWheel::addReceiver( std::shared_ptr<WheelOdomReceiver> & receiver )
{
   receivers.push_back( receiver );
}
