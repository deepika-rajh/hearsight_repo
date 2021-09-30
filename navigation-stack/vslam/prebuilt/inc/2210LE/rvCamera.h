/*****************************************************************************
@copyright
Copyright (c) 2021 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef _ROBOT_VISION_CAMERA_H_
#define _ROBOT_VISION_CAMERA_H_

#include "rv.h"

struct rvCameraParams
{
   typedef enum _DistortionModel
   {
      RationalModel_12 = 0,
      FisheyeModel_4,
      NoDistortion
   } rvDistortionModel;
    
   int32_t inputPixelWidth;
   int32_t inputPixelHeight;
   int32_t outputPixelWidth;
   int32_t outputPixelHeight;

   rvDistortionModel distortionModel;
   float32_t inputCameraMatrix[9];
   float32_t distortionCoefficient[12];
   float32_t outputCameraMatrix[9];
};

#endif
