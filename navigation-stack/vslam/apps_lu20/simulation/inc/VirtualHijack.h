/***************************************************************************//**
@copyright
Copyright (c) 2017-2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef __VIRTUAL_HIJACK_H__
#define __VIRTUAL_HIJACK_H__

#include <fstream>
#include "rvQueue.h"
#include "hijack_datatype.h"


class VirtualHijack
{
public:
   VirtualHijack( );
   ~VirtualHijack();

   static bool GetHijack( const sensor_hijack & hijack );

};
#endif
