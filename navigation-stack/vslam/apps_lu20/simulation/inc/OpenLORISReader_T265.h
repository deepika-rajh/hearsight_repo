/*******************************************************************************
@copyright
Copyright (c) 2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef __OPENLORIS_READER_T265_H__

#define __OPENLORIS_READER_T265_H__


#include "OpenLORISReader.h"

#include <string>

class OpenLORISReaderT265 : public OpenLORISReader
{
public:
   OpenLORISReaderT265(const std::string & imageListFile );
   ~OpenLORISReaderT265();
   virtual bool GetNextFrame( mvFrame & frame, std::vector<sensor_wheel> & wheelOdomSet, std::vector<imu_pack_dsp> & imuSampleSet, std::vector<sensor_hijack> & hijackSet, std::vector<StampedSystemCallback> & callbackSet, std::vector<rvPose6DRTWithTimestamp>& poses);
   bool getCameraConfiguration( rvCameraParams & config );
private:
 
   void ReadCameraConfig( const std::string & imageListFile, rvStereoCamera & cameraConfig );

   rvStereoCamera cameraConfig;

};

#endif
