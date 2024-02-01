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

#include "sensor_client.h"

class IMURecorder
{
public:
   IMURecorder(const char * path);
   ~IMURecorder();
   void Record( sensors_event_t * accel_ptr, sensors_event_t * gyro_ptr, size_t pack_num);

private:
   FILE * aFp = NULL;
   FILE * gFp = NULL;
};

#endif
