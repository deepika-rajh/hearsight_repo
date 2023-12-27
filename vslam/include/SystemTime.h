/*****************************************************************************
@copyright
Copyright (c) 2020-2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef __SYSTEM_TIME_H__

#define __SYSTEM_TIME_H__

#include <stdint.h>

int64_t getRealTime();
int64_t getMonotonicTime();
double getDspClock();
int64_t getTimeEpoch();

#endif //__SYSTEM_TIME_H__
