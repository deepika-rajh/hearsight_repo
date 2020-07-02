/*****************************************************************************
@copyright
Copyright (c) 2020 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/
#include <functional>
#include "VSLAMHijack.h"
#include "hijack_datatype.h"
#include "string.h"
#include "VSLAMSystem.h"
#include "Queue.h"

#define BUF_SIZE 10
queue_mt<sensor_hijack> hijackArray( BUF_SIZE );

void VSLAMHijack::hijackProc()
{
   int numSamples = 0;
   float delta = 0.f;
   sensor_hijack * sensorDataPtr = new sensor_hijack[BUF_SIZE];
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
         sensor_hijack * curData = (sensor_hijack *)sensorDataPtr + j;
         int64_t curTimeStampNs = (int64_t)curData->timestamp;
         if( lastTimeStamp != 0 )
         {
            delta = (curTimeStampNs - lastTimeStamp)*1e-6f;
         }
         lastTimeStamp = curTimeStampNs;

         for( std::shared_ptr<HijackReceiver> receiver : receivers )
         {
            receiver->addHijack( curData->hijackStatus, curTimeStampNs / 1000 );
         }
      }

      // Sleep to avoid the crash caused by destroying the mutex while its busy
      // Would not a good idea and just a workaround
      //VSLAM_MASTER_SLEEP( 20 );
   }
   delete[]( sensor_hijack * )sensorDataPtr;

   printf("6. ----------> hijackProc thread exit \n");
}

VSLAMHijack::VSLAMHijack()
{
   running = false;
}

VSLAMHijack::~VSLAMHijack()
{
}


void VSLAMHijack::stop()
{
	running = false;
   sensor_hijack hijack;
   hijack.hijackStatus = false;
   hijack.timestamp = 0;
   hijackArray.check_push( hijack );
   if( hijackPullThread )
   {
      hijackPullThread->join();
   }
}

void VSLAMHijack::start()
{
   running = true;
   hijackPullThread = std::make_shared<std::thread>(std::mem_fn(&VSLAMHijack::hijackProc), this);
}

int16_t VSLAMHijack::getData( sensor_hijack* dataArray, int32_t max_count, int32_t* available_data_num )
{
   int32_t i;
   for( i = 0; i < max_count; i++ )
   {
      sensor_hijack hijack;
      bool result = hijackArray.try_pop( hijack );
      if( !result )
         hijackArray.wait_and_pop( hijack );

      //{
         dataArray[i].timestamp = hijack.timestamp;
         dataArray[i].hijackStatus = hijack.hijackStatus;
      //}
      //else
      //{
      //   break;
      //}
   }
   *available_data_num = i;
   return 1;
}

void VSLAMHijack::addReceiver( std::shared_ptr<HijackReceiver> & receiver )
{
   receivers.push_back( receiver );
}
