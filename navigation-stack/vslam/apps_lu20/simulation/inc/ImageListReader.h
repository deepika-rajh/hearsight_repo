/*******************************************************************************
@copyright
Copyright (c) 2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef __IMAGE_LIST_READER_H__

#define __IMAGE_LIST_READER_H__

#include "SlamDataReader.h"

#include <string>

class ImageListReader : public SlamDataReader
{
public:
   ImageListReader(const std::string & imageListFile, const std::string & wheelFile, const std::string & hijackName, const std::string & callbackName );
   ~ImageListReader();
   virtual bool SkipNextFrame();
   virtual bool GetNextFrame( mvFrame & frame, std::vector<sensor_wheel> & wheelOdomSet, std::vector<imu_pack_dsp> & imuSampleSet, std::vector<sensor_hijack> & hijackSet, std::vector<StampedSystemCallback> & callbackSet, std::vector<rvPose6DRTWithTimestamp>& poses);
   bool getCameraConfiguration( rvCameraParams & config )
   {
      return false;
   }
   bool getIMUConfiguration( rvIMUConfiguration & config )
   {
      return false;
   }
   bool getWheelConfiguration( rvWheelConfiguration & config )
   {
      return false;
   }
   bool getTargetImage(rvTargetImage & image)
   {
       return false;
   }
private:
   
   void ReadImageList( const std::string & imageListFile, std::vector<std::string> & imageList );

   std::vector<std::string> imageList;
   size_t curImageIndex;
};

#endif
