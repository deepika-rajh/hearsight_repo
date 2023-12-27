/*****************************************************************************
@copyright
Copyright (c) 2020-2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include "IMURecorder.h"

#include <string>

IMURecorder::IMURecorder( const char * path )
{
   float accVal[3], gyrVal[3];
   float delta = 0.f;
   int64_t lastTimeStamp = 0;
   const char head0[] = "<?xml version='1.0' encoding='UTF-8'?>";
   const char head1[] = "<Sequence>";
   const char head2[] = "  <Dataset>";
   std::string pathS( path );
   std::string fileName = pathS + "/accelerometer.xml";
   aFp = fopen( fileName.c_str(), "wt" );
   fileName = pathS + "/gyroscope.xml";
   gFp = fopen( fileName.c_str(), "wt" );
   if( aFp )
   {
      fprintf( aFp, "%s\n%s\n%s\n", head0, head1, head2 );
      fprintf( gFp, "%s\n%s\n%s\n", head0, head1, head2 );
   }
}

void IMURecorder::Record( sensors_event_t * accel_ptr, sensors_event_t * gyro_ptr, size_t pack_num )
{
   for( int j = 0; j < pack_num; ++j )
   {
      int64_t curTimeStampNs = accel_ptr->timestamp;
      if( aFp )
      {
         fprintf( aFp, "    <Data x='%.6f' y='%.6f' z='%.6f' timestamp='%ld'/>\n",
                  accel_ptr->acceleration.x, accel_ptr->acceleration.y, accel_ptr->acceleration.z, curTimeStampNs );
         fprintf( gFp, "    <Data x='%.6f' y='%.6f' z='%.6f' timestamp='%ld'/>\n",
                  gyro_ptr->gyro.x, gyro_ptr->gyro.y, gyro_ptr->gyro.z, curTimeStampNs );
      }
	  accel_ptr += 1;
      gyro_ptr += 1;
   }
}

IMURecorder::~IMURecorder()
{
   const char tail0[] = "  </Dataset>";
   const char tail1[] = "</Sequence>";
   if( aFp )
   {
      fprintf( aFp, "%s\n%s\n", tail0, tail1 );
      fclose( aFp );
      fprintf( gFp, "%s\n%s\n", tail0, tail1 );
      fclose( gFp );
   }
}
