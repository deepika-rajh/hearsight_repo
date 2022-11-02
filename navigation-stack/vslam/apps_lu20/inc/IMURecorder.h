/*****************************************************************************
@copyright
Copyright (c) 2020-2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/
#ifndef _IMU_RECORDER_H_
#define _IMU_RECORDER_H_

#include "stdio.h"
#include "stdint.h"

struct imu_pack_dsp;

class IMURecorder
{
public:
   IMURecorder(const char * path);
   ~IMURecorder();
   void Record( struct imu_pack_dsp * imu_data, int64_t clockOffset, size_t pack_num );

private:
   FILE * aFp = NULL;
   FILE * gFp = NULL;
};

#endif
