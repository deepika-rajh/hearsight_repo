/*******************************************************************************
@copyright
Copyright (c) 2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include "SlamDataReader.h"

SlamDataReader::SlamDataReader( const std::string & wheelOdomName, const std::string & hijackName, const std::string & callbackName, const std::string& poseFile)
{
   mWheel = new WheelReader( wheelOdomName );
   mHijack = new HijackReader( hijackName );
   mCallbackReader = new CallbackReader( callbackName );
   mPoseSensor = new PoseReader(poseFile);
}

SlamDataReader::~SlamDataReader()
{
   if( mWheel )
   {
      delete mWheel;
      mWheel = NULL;
   }

   if( mHijack )
   {
      delete mHijack;
      mHijack = NULL;
   }

   if( mCallbackReader )
   {
      delete mCallbackReader;
      mCallbackReader = NULL;
   }
}

void SlamDataReader::GetWheelOdom( uint64_t timestamp, std::vector<sensor_wheel> & wheelOdomSet )
{   
   sensor_wheel wheelOdom;
   wheelOdomSet.clear();
   do
   {
      bool isWheelodomValid = mWheel->GetWheelOdom( timestamp, wheelOdom );
      if( isWheelodomValid )
      {
         if( wheelOdom.timestamp != 0 )
         {
            if ( wheelOdom.timestamp > timestamp - 100e6 )
               wheelOdomSet.push_back( wheelOdom );
         }
         else
         {
            break;
         }
      }
      else
      {
         printf( "Wheel odom reading error\n" );
         break;
      }
   } while( wheelOdom.timestamp <= timestamp );
}



void SlamDataReader::GetPose(uint64_t timestamp, std::vector<rvPose6DRTWithTimestamp>& poseSet)
{
    rvPose6DRTWithTimestamp pose;
    poseSet.clear();
    do
    {
        bool isPoseValid = mPoseSensor->getPose(timestamp, pose);
        if (isPoseValid)
        {
            if (pose.timestamp != 0)
            {
                if (pose.timestamp > timestamp - 100e6)
                    poseSet.push_back(pose);
            }
            else
            {
                break;
            }
        }
        else
        {
            printf("pose reading error\n");
            break;
        }
    } while (pose.timestamp <= timestamp);
}


void SlamDataReader::GetHijack( uint64_t timestamp, std::vector<sensor_hijack> & hijackSet )
{
   sensor_hijack hijack;
   hijackSet.clear();
   do
   {
      bool isHijackValid = mHijack->getHijack( timestamp, hijack );
      if( isHijackValid )
      {
         if( hijack.timestamp != 0 )
         {
            if ( hijack.timestamp > timestamp - 100e6 )
               hijackSet.push_back( hijack );
         }
         else
         {
            break;
         }
      }
      else
      {
         printf( "Hijack reading error\n" );
         break;
      }
   } while( hijack.timestamp <= timestamp );
}


void SlamDataReader::GetCallback( uint64_t timestamp, std::vector<StampedSystemCallback> & callbackSet )
{
   StampedSystemCallback callback;
   callbackSet.clear();
   do
   {
      bool isCallbackValid = mCallbackReader->getCallback( timestamp, callback );
      if( isCallbackValid )
      {
         if( callback.timestamp != 0 )
         {
            if( callback.timestamp > timestamp - 100e6 )
               callbackSet.push_back( callback );
         }
         else
         {
            break;
         }
      }
      else
      {
         printf( "Callback reading error\n" );
         break;
      }
   } while( callback.timestamp <= timestamp );
}

void SlamDataReader::ReleaseMVImage( mvImage * image )
{
   if( image->pixels != NULL )
   {
      delete image->pixels;
      image->pixels = NULL;
      image->height = 0;
      image->memoryStride = 0;
      image->width = 0;
   }
}

void SlamDataReader::ReleaseMVImage( mvImage16 * image )
{
   if( image->pixels != NULL )
   {
      delete image->pixels;
      image->pixels = NULL;
      image->height = 0;
      image->memoryStride = 0;
      image->width = 0;
   }
}

void SlamDataReader::AllocateMvImage( mvImage * image, size_t width, size_t height, size_t stride )
{
   image->height = height;
   image->width = width;
   image->memoryStride = stride;
   image->pixels = new uint8_t[stride*height];
}

void SlamDataReader::AllocateMvImage( mvImage16 * image, size_t width, size_t height )
{
   image->height = height;
   image->width = width;
   image->memoryStride = sizeof( uint16_t )*width;
   image->pixels = new uint16_t[width*height];
}
