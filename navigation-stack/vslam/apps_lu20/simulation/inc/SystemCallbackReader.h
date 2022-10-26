/*******************************************************************************
@copyright
Copyright (c) 2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef __CALLBACK_READER_H__
#define __CALLBACK_READER__H__


#include <fstream>

typedef enum
{
   KNONE = 0,
   KSLEEP,
   KAWAKE,
   KRESET,
   KSTOP
} SystemCallback;

typedef struct
{
   SystemCallback callback;
   uint64_t       timestamp;
} StampedSystemCallback;

class CallbackReader
{
public:
   CallbackReader(const std::string & wheelOdomName );
   ~CallbackReader();

   bool getCallback( uint64_t timestamp, StampedSystemCallback & callback );

   bool callbackFileReady() const;

private:
   bool getCallback( StampedSystemCallback & callback );
   std::ifstream callbackStream;
   StampedSystemCallback curCallback;
   bool fileReady;
};
#endif
