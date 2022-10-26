/*******************************************************************************
@copyright
Copyright (c) 2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef __OPENLORIS_READER_D435I_H__

#define __OPENLORIS_READER_D435I_H__

#include "OpenLORISReader.h"

#include <string>

class OpenLORISReaderD435i : public OpenLORISReader
{
public:
   OpenLORISReaderD435i(const std::string & imageListFile );
   ~OpenLORISReaderD435i();

   virtual bool GetNextFrame( mvFrame & frame, std::vector<sensor_wheel> & wheelOdomSet, std::vector<imu_pack_dsp> & imuSampleSet, std::vector<sensor_hijack> & hijackSet, std::vector<StampedSystemCallback> & callbackSet, std::vector<rvPose6DRTWithTimestamp>& poses);
   bool getCameraConfiguration( rvCameraParams & config );

private:

   rvCameraIntrinsic cameraConfig;

};

#endif
