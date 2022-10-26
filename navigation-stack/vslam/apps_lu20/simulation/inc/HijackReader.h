/*******************************************************************************
@copyright
Copyright (c) 2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef __HIJACK_READER_H__
#define __HIJACK_READER__H__


#include <fstream>
#include "hijack_datatype.h"

class HijackReader
{
public:
   HijackReader(const std::string & wheelOdomName );
   ~HijackReader();

   bool getHijack( uint64_t timestamp, sensor_hijack & hijack );

   bool hijackFileReady() const;

private:
   std::ifstream hijackStream;  
   sensor_hijack hijackBackup;
   bool getHijack( sensor_hijack & hijack );

   bool fileReady;
};
#endif
