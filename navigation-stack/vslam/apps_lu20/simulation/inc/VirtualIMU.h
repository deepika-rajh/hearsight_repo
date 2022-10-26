/*****************************************************************************
* @copyright
* Copyright (c) 2018-2022 Qualcomm Technologies, Inc.
* All Rights Reserved.
* Confidential and Proprietary - Qualcomm Technologies, Inc.
*********************************************************************************/
#ifndef __VIRTUAL_IMU_H__
#define __VIRTUAL_IMU_H__

#include "rvQueue.h"

struct imu_pack_dsp;

class VirtualIMU
{
public:
   VirtualIMU( );
   ~VirtualIMU();

   static bool GetIMUData( const imu_pack_dsp & );
   static bool OutputData( imu_pack_dsp & );

protected:
   static queue_mt<imu_pack_dsp> dataArray;
};
#endif
