/***************************************************************************//**
@copyright
Copyright (c) 2017-2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include "VirtualHijack.h"

#include <string>
#include "math.h"


extern queue_mt<sensor_hijack> hijackArray;

VirtualHijack::VirtualHijack( )
{

}

VirtualHijack::~VirtualHijack( )
{

}

bool VirtualHijack::GetHijack( const sensor_hijack & hijack )
{
   hijackArray.check_push( hijack );
   return true;
}




