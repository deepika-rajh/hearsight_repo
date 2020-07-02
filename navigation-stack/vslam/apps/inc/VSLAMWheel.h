/*****************************************************************************
@copyright
Copyright (c) 2020 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/
#ifndef _VSLAM_WHEEL_H_
#define _VSLAM_WHEEL_H_

#include "wheel_datatype.h"
#include <vector>
#include <thread> 
#include <memory>

class WheelOdomReceiver
{
public:
   virtual void addWheelOdom( float linearVelocity, float angualVelocity,
                      const float location[3], const float direction[4], int64_t timestamp ) = 0;
};

class VSLAMWheel
{
public:
   VSLAMWheel();
   ~VSLAMWheel();
   void addReceiver(std::shared_ptr<WheelOdomReceiver> & receiver);

   void stop();
   void start();

   int16_t getData( sensor_wheel* dataArray, int32_t max_count, int32_t* available_num );

protected:
   VSLAMWheel(const VSLAMWheel &) = delete;
   VSLAMWheel &operator = (const VSLAMWheel &) = delete;

   void wheelProc();
   bool running;
   std::vector<std::shared_ptr<WheelOdomReceiver>> receivers;

   std::shared_ptr<std::thread> wheelPullThread;
};

#endif //_VSLAM_WHEEL_H_
