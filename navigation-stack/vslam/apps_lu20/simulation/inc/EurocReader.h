/*******************************************************************************
@copyright
Copyright (c) 2022-2023 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef __EUROC_READER_H__

#define __EUROC_READER_H__

#include "SlamDataReader.h"

#include <string>

class EurocReader : public SlamDataReader
{
public:
   EurocReader(const std::string & imageListFile );
   ~EurocReader();
   virtual bool SkipNextFrame();
   virtual bool GetNextFrame( mvFrame & frame, std::vector<sensor_wheel> & wheelOdomSet, std::vector<imu_pack_dsp> & imuSampleSet, std::vector<sensor_hijack> & hijackSet, std::vector<StampedSystemCallback> & callbackSet, std::vector<rvPose6DRTWithTimestamp>& poses);
   bool getCameraConfiguration( rvCameraParams & config );
   bool getIMUConfiguration( rvIMUConfiguration & config );
   bool getWheelConfiguration( rvWheelConfiguration & config );

   bool getTargetImage(rvTargetImage & target)
   {       
       return false;
   }

private:

   void ReadImageList( const std::string & imageListFile, std::vector<StereoSample> & imageList );
   void LoadFileSamples( const std::string & filename, std::vector<fileWithTimestamp>& sampleSet );
   void ReadStereoCameraConfig( const std::string & imageListFile, rvStereoCamera & cameraConfig, rvPose6DRT & imuCamera0 );
   void ReadCameraConfig( const std::string & imageListFile, rvCameraIntrinsic & cameraConfig, rvPose6DRT & bodyCamera );

   void ReadIMUSamples( const std::string & imageListFile, std::vector<imu_pack_dsp> & imuSampleSet );

   std::vector<StereoSample> imageList;
   size_t curImageIndex;
   std::vector<imu_pack_dsp> imuSamples;
   size_t curIMUIndex;

   rvStereoCamera cameraConfig;

   rvPose6DRT imuCamera0;
};

#endif
