/*******************************************************************************
@copyright
Copyright (c) 2017-2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include "HijackReader.h"
#include <string>


HijackReader::HijackReader( const std::string & hijackName )
{
   fileReady = false;
   hijackStream.open( hijackName.c_str() );
   if( !hijackStream.is_open() )
   {
      printf( "Cannot open file %s for hijack status reading!\n", hijackName.c_str() );
   }
   else
   {
      fileReady = true;
      printf( "Open file %s for hijack status reading!\n", hijackName.c_str() );
   }
   hijackBackup.timestamp = 0;
}

HijackReader::~HijackReader()
{

}


bool HijackReader::hijackFileReady() const
{
   return fileReady;
}


bool HijackReader::getHijack( uint64_t timestamp, sensor_hijack & hijack )
{
   bool result = true;
   if( hijackBackup.timestamp == 0 )
   {
      result = getHijack( hijackBackup );
   }
   
   if( hijackBackup.timestamp < timestamp && result )
   {
      hijack = hijackBackup;
      hijackBackup.timestamp = 0;
   }
   else
   {
      hijack.timestamp = 0;
   }

   return result;
}

bool HijackReader::getHijack( sensor_hijack & hijack )
{
   if( hijackStream.eof() )
   {
      return false;
   }

   if( hijackStream >> hijack.timestamp )
   {
      std::string hijackStatus;
      hijackStream >> hijackStatus;

      if( hijackStatus == "Hijack1" )
         hijack.hijackStatus = true;
      else
         hijack.hijackStatus = false;

      char linebreak = hijackStream.get();
      if( '\n' != linebreak )
      {
         printf( "Unsupported data format for hijack (LF expected but %c received)!\n", linebreak );
         return false;
      }
   }
   else
   {
      printf( "Cannot read out hijack timestamp from file!\n" );
      return false;
   }

   return true;
}


