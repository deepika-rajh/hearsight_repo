/*******************************************************************************
@copyright
Copyright (c) 2017-2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include "WheelReader.h"
#include <string>


WheelReader::WheelReader( const std::string & wheelOdomName )
{
   wheelodomStream.open( wheelOdomName.c_str() );
   if( !wheelodomStream.is_open() )
   {
      printf( "Cannot open file %s for wheel odom reading!\n", wheelOdomName.c_str() );
   }
   else
   {
      printf( "Open file %s for wheel odom reading!\n", wheelOdomName.c_str() );
   }
   wheelOdomBackup.timestamp = 0;
}

WheelReader::~WheelReader()
{

}

bool WheelReader::GetWheelOdom( uint64_t timestamp, sensor_wheel & wheelodom )
{
   bool result = true;
   if( wheelOdomBackup.timestamp == 0 )
   {
      result = GetWheelOdom( wheelOdomBackup );
   }
   
   if( wheelOdomBackup.timestamp < timestamp && result )
   {
      wheelodom = wheelOdomBackup;
      wheelOdomBackup.timestamp = 0;
   }
   else
   {
      wheelodom.timestamp = 0;
   }

   return result;
}

bool WheelReader::GetWheelOdom( sensor_wheel & wheelodom )
{
   if( wheelodomStream.eof() )
   {
      return true;
   }

   if( wheelodomStream >> wheelodom.timestamp )
   {
      wheelodomStream >> wheelodom.location[0] >> wheelodom.location[1] >> wheelodom.location[2];
      wheelodomStream >> wheelodom.direction[0] >> wheelodom.direction[1] >> wheelodom.direction[2] >> wheelodom.direction[3];
 
      double velocityNotUsed;
      wheelodomStream >> wheelodom.linear_velocity;
      wheelodomStream >> velocityNotUsed; wheelodomStream >> velocityNotUsed; // velocityLinear along y and z axis
      wheelodomStream >> velocityNotUsed; wheelodomStream >> velocityNotUsed; // velocityAngular along x and y axis
      wheelodomStream >> wheelodom.angular_velocity;

      char linebreak = wheelodomStream.get();
      if( '\n' != linebreak )
      {
         printf( "Unsupported data format for wheelodom (LF expected but %c received)!\n", linebreak );
         return false;
      }
   }
   else
   {
      printf( "Cannot read out wheel timestamp from file!\n" );
      return false;
   }

   return !wheelodomStream.eof();
}


