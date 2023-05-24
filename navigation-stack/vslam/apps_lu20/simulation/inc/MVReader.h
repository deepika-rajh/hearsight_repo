/*****************************************************************************
 * @copyright
 * Copyright (c) 2018-2023 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 * *******************************************************************************/

#ifndef __MV_READER_H__

#define __MV_READER_H__


#include "SlamDataReader.h"
#include "rvCamera.h"
#include <string>

#include "mvSRW.h"

class MVReader : public SlamDataReader
{
public:
   MVReader( const std::string & imageListFile );
   ~MVReader();
   bool getCameraConfiguration(rvCameraParams & );
   virtual bool SkipNextFrame();
   virtual bool GetNextFrame( mvFrame & frame, std::vector<sensor_wheel> & wheelOdomSet, std::vector<imu_pack_dsp> & gyroSet, std::vector<sensor_hijack> & hijackSet, std::vector<StampedSystemCallback> & callbackSet, std::vector<rvPose6DRTWithTimestamp>& poses);
   bool getIMUConfiguration( rvIMUConfiguration & config )
   {
       config = imuCon;
      return true;
   }
   bool getWheelConfiguration( rvWheelConfiguration & config );

   bool getTargetImage(rvTargetImage & target)
   {
       target = targetImage;
       return true;
   }

private:
   mvSRW_Reader * reader;
   bool GetNextIMUSample( mvSRW_Reader * sequenceReader, uint64_t time, imu_pack_dsp & imuSample );
   void fillOneImage( const mvImage * src, mvImage * dst );
   void ReadWheelConfiguration( std::string & fileName );
   bool ParseCameraParameters( const std::string & root, const std::string & configFile );
   bool GetCameraParameter( const char *cameraID, mvCameraConfiguration & configuration, mvCameraConfiguration & outputCamera );
   void getCameraSetting( const cv::Mat & intrinsics, const std::string & distortionModelName, const cv::Mat & distortion, const cv::Size & imageSize, mvCameraConfiguration & cameraConfig );

   mvCameraConfiguration inputCamera, outputCamera;

   rvStereoCamera stereoCamera;
   rvStereoRectCamera rectStereoCamera;
   bool configValid;

   mvCameraDescriptor * cameras;

   rvPose6DRT imuCamera;
   bool imuReady;
   rvWheelConfiguration wheelCon;
   rvIMUConfiguration imuCon;
   rvTargetImage targetImage;
};

#endif
