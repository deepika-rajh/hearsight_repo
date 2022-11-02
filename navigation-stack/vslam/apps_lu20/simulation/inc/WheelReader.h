/*******************************************************************************
@copyright
Copyright (c) 2017-2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef __WHEEL_READER_H__
#define __WHEEL_READER__H__


#include <fstream>
#include "wheel_datatype.h"

class WheelReader
{
public:
   WheelReader(const std::string & wheelOdomName );
   ~WheelReader();

   bool GetWheelOdom( uint64_t timestamp, sensor_wheel & wheelodom );

private:
   std::ifstream wheelodomStream;  
   sensor_wheel wheelOdomBackup;
   bool GetWheelOdom( sensor_wheel & wheelodom );
};
#endif
