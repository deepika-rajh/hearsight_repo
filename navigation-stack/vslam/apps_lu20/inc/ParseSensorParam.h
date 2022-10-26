/*****************************************************************************
@copyright
Copyright (c) 2020-2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef __PARSE_SENSOR_PARAM_H__

#define __PARSE_SENSOR_PARAM_H__

#include "rvCamera.h"
#include <string>

bool ParseSensorParam( const std::string & root, const std::string & configFile, rvWheelConfiguration & wheelConf, rvIMUConfiguration & imuConf, rvTargetImage & targetImage);
void ReadMatrix( std::ifstream & file, float * matrix );

#endif //__PARSE_SENSOR_PARAM_H__
