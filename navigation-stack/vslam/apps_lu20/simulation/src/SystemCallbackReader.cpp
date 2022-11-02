/*******************************************************************************
@copyright
Copyright (c) 2017-2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include "SystemCallbackReader.h"
#include <string>


CallbackReader::CallbackReader( const std::string & callbackName )
{
   fileReady = false;

   callbackStream.open( callbackName.c_str() );
   if( !callbackStream.is_open() )
   {
      printf( "Cannot open file %s for hijack status reading!\n", callbackName.c_str() );
   }
   else
   {
      printf( "Open file %s for hijack status reading!\n", callbackName.c_str() );
      fileReady = true;
   }
   curCallback.callback = KNONE;
   curCallback.timestamp = 0;
}

CallbackReader::~CallbackReader()
{

}


bool CallbackReader::callbackFileReady() const
{
   return fileReady;
}


bool CallbackReader::getCallback( uint64_t timestamp, StampedSystemCallback & callback )
{
   bool result = true;
   if( curCallback.timestamp == 0 )
   {
      result = getCallback( curCallback );
   }
   
   if( curCallback.timestamp < timestamp && result )
   {
      callback = curCallback;
      curCallback.timestamp = 0;
   }
   else
   {
      callback.timestamp = 0;
   }

   return result;
}

bool CallbackReader::getCallback( StampedSystemCallback & callback )
{
   if( callbackStream.eof() )
   {
      return false;
   }

   if( callbackStream >> callback.timestamp )
   {
      std::string callbackString;
      callbackStream >> callbackString;

      if( callbackString == "sleep" )
         callback.callback = KSLEEP;
      else if( callbackString == "awake" )
         callback.callback = KAWAKE;
      else if (callbackString == "reset" )
         callback.callback = KRESET;
      else
      {
         printf( "undefined callback type\n" );
      }

      char linebreak = callbackStream.get();
      if( '\n' != linebreak )
      {
         printf( "Unsupported data format for callback (LF expected but %c received)!\n", linebreak );
         return false;
      }
   }
   else
   {
      printf( "Cannot read out callback timestamp from file!\n" );
      return false;
   }

   return true;
}


