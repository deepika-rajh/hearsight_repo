/*******************************************************************************
@copyright
Copyright (c) 2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef __OPENLORIS_READER_H__

#define __OPENLORIS_READER_H__

#include "SlamDataReader.h"

#include <string>

class OpenLORISReader : public SlamDataReader
{
public:
   OpenLORISReader(const std::string & imageListFile );
   ~OpenLORISReader();
   virtual bool SkipNextFrame();
   bool getIMUConfiguration( rvIMUConfiguration & config );
   bool getWheelConfiguration( rvWheelConfiguration & config );

   bool getTargetImage(rvTargetImage & target)
   {
       return false;
   }
protected:

   struct RGBDSample
   {
      int64_t timestamp;
      std::string grayImageName;
      std::string depthImageName;
   };

   struct fileWithTimestamp
   {
      int64_t timestamp;
      std::string filename;
   };
   
   void ReadImageList( const std::string & imageListPath, const std::string & leftList, const std::string & rightList, std::vector<RGBDSample> & imageList );
   void LoadFileSamples( const std::string & filename, std::vector<fileWithTimestamp>& sampleSet );   
   void ReadMonoCameraConfig( const std::string & imageListFile, const std::string & cameraName, rvCameraIntrinsic & cameraConfig );
   void ReadExtrinsicParams( const std::string & transFile, 
                             const std::string & cameraFrameL, 
                             const std::string & cameraFrameR,
                             const std::string & imuFrame,
                             rvPose6DRT & imuCamera, 
                             rvPose6DRT & imuWheel,
                             rvPose6DRT & stereoCameraExtrinsic);

   void ReadIMUSamples( const std::string & imageListFile, const std::string & accFile, const std::string & gyroFile,
                        std::vector<imu_pack_dsp> & imuSampleSet );
   typedef struct
   {
      double timestamp;
      float32_t x, y, z;
   } SensorData;
   void LoadSensor( const std::string & imageListFile, std::vector<SensorData> & sensorSampleSet );

   void ReadOdomSamples( const std::string & imageListFile, std::vector<sensor_wheel> & wheelSampleSet );

   std::vector<RGBDSample> imageList;
   size_t curImageIndex;
   std::vector<imu_pack_dsp> imuSamples;
   size_t curIMUIndex;
   std::vector<sensor_wheel> wheelSamples;
   size_t curWheelIndex;

   rvPose6DRT imuCamera;
   rvPose6DRT cameraBaselink;
   rvPose6DRT cameraLCameraR;
};

#endif
