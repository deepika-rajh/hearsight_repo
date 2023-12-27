/*****************************************************************************
@copyright
Copyright (c) 2020-2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef _VSLAM_HIJACK_H_
#define _VSLAM_HIJACK_H_

#include "hijack_datatype.h"
#include <vector>

#include <thread> 
#include <memory>

class HijackReceiver
{
public:
   virtual void addHijack( bool status, int64_t timestamp ) = 0;
};

class VSLAMHijack
{
public:
   VSLAMHijack();
   ~VSLAMHijack();
   void addReceiver(std::shared_ptr<HijackReceiver> & receiver);

   void stop();
   void start();

   int16_t getData( sensor_hijack* dataArray, int32_t max_count, int32_t* available_num );

protected:
   VSLAMHijack(const VSLAMHijack &) = delete;
   VSLAMHijack &operator = (const VSLAMHijack &) = delete;

   void hijackProc();
   bool running;
   std::vector<std::shared_ptr<HijackReceiver>> receivers;

   std::shared_ptr<std::thread> hijackPullThread;
};

#endif //_VSLAM_WHEEL_H_
