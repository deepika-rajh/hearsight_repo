/*******************************************************************************
@copyright
Copyright (c) 2022-2023 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef __SLAM_DATA_READER_H__

#define __SLAM_DATA_READER_H__
#include "rv.h"
#include "WheelReader.h"
#include "HijackReader.h"
#include "SystemCallbackReader.h"
#include "PoseReader.h"
#include "imu_client.hpp"
#include "PoseReader.h"

#include <vector>
#include "mvSRW.h"

#include "rvCamera.h"

class SlamDataReader
{
public:
   SlamDataReader( const std::string & wheelOdomName, const std::string & hijackName, const std::string & callbackName, const std::string & poseFile );
   
   virtual ~SlamDataReader();
   virtual bool SkipNextFrame() = 0;
   virtual bool GetNextFrame( mvFrame & frame, std::vector<sensor_wheel> & wheelOdomSet, std::vector<imu_pack_dsp> & imuSampleSet, std::vector<sensor_hijack> & hijackSet, std::vector<StampedSystemCallback> & callbackSet, std::vector<rvPose6DRTWithTimestamp> & poses ) = 0;
   virtual bool getCameraConfiguration( rvCameraParams & config) = 0;
   virtual bool getIMUConfiguration( rvIMUConfiguration & config ) = 0;
   virtual bool getWheelConfiguration( rvWheelConfiguration & config ) = 0;
   virtual bool getTargetImage(rvTargetImage & targetImagePath) = 0;
protected:
   void GetWheelOdom( uint64_t timestamp, std::vector<sensor_wheel> & wheelOdomSet );
   WheelReader * mWheel;

   void GetHijack( uint64_t timestamp, std::vector<sensor_hijack> & hijackSet );
   HijackReader * mHijack;

   void GetCallback( uint64_t timestamp, std::vector<StampedSystemCallback> & hijackSet );
   CallbackReader * mCallbackReader;

   void GetPose(uint64_t timestamp, std::vector< rvPose6DRTWithTimestamp>& poseSet);
   PoseReader* mPoseSensor;

   void ReleaseMVImage( mvImage * image );
   void ReleaseMVImage( mvImage16 * image );
   void AllocateMvImage( mvImage * image, size_t width, size_t height, size_t stride );
   void AllocateMvImage( mvImage16 * image, size_t width, size_t height );
};

#include "opencv2/opencv.hpp"
template<typename type>
void copyFromMat( const cv::Mat & s, rvPose6DRT &d )
{
   for( size_t i = 0; i < 3; i++ )
      for( size_t j = 0; j < 4; j++ )
         d.matrix[i][j] = (float32_t) s.at<type>( i, j );
}

struct StereoSample
{
   int64_t timestamp;
   std::string leftImageName;
   std::string rightImageName;
};

struct fileWithTimestamp
{
   int64_t timestamp;
   std::string filename;
};



#endif //__SLAM_DATA_READER_H__
