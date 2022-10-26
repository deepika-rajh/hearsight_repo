/*****************************************************************************
@copyright
Copyright (c) 2020-2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/
#include "VSLAMWheel.h"
#include "wheel_datatype.h"
#include "string.h"
#include <functional>
//#include "rvDebugger.h"

bool VSLAMWheel::running;
std::vector<std::shared_ptr<WheelOdomReceiver>> VSLAMWheel::receivers;

void VSLAMWheel::wheelCallback(const sensor_wheel * curData)
{
    if (!running)
        return;   

	for (std::shared_ptr<WheelOdomReceiver> receiver : receivers)
	{
		receiver->addWheelOdom(curData->linear_velocity,
			curData->angular_velocity,
			curData->location,
			curData->direction, curData->timestamp);
	}
}

VSLAMWheel::VSLAMWheel()
{
   running = false;
}

VSLAMWheel::~VSLAMWheel()
{
}


void VSLAMWheel::stop()
{
	running = false;
}

void VSLAMWheel::start()
{
   running = true;
}

      //RV_INFO("iterator %d, wheelDataArray poped", i);
void VSLAMWheel::addReceiver( std::shared_ptr<WheelOdomReceiver> & receiver )
{
   receivers.push_back( receiver );
}
