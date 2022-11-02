/*******************************************************************************
@copyright
Copyright (c) 2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef __TUM_RGBD_READER_H__

#define __TUM_RGBD_READER_H__

#include "SlamDataReader.h"

#include <string>

class TUMRGBDReader : public SlamDataReader
{
public:
   TUMRGBDReader(const std::string & imageListFile );
   ~TUMRGBDReader();
   virtual bool SkipNextFrame();
   virtual bool GetNextFrame( mvFrame & frame, std::vector<sensor_wheel> & wheelOdomSet, std::vector<imu_pack_dsp> & imuSampleSet, std::vector<sensor_hijack> & hijackSet, std::vector<StampedSystemCallback> & callbackSet, std::vector<rvPose6DRTWithTimestamp>& poses);
   bool getCameraConfiguration( rvCameraParams & config );

   bool getIMUConfiguration( rvIMUConfiguration & config )
   {
      return false;
   }
   bool getWheelConfiguration( rvWheelConfiguration & config )
   {
      return false;
   }
   bool getTargetImage(rvTargetImage& target)
   {
       return false;
   }
private:

   size_t cameraIdx;

   struct RGBDSample
   {
      double timestamp;
      std::string rgbImageName;
      std::string depthImageName;
   };

   struct fileWithTimestamp
   {
      double timestamp;
      std::string filename;
   };
   
   void ReadImageList( const std::string & imageListFile, std::vector<RGBDSample> & imageList );
   void LoadFileSamples( const std::string & filename, std::vector<fileWithTimestamp>& sampleSet );
   std::vector<RGBDSample> imageList;
   size_t curImageIndex;
};

#endif
