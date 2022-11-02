/*******************************************************************************
@copyright
Copyright (c) 2017-2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include "VirtualIMU.h"

#include "imu_client.hpp"
queue_mt<imu_pack_dsp> VirtualIMU::dataArray( 300 );


VirtualIMU::VirtualIMU(  )
{
   
}

VirtualIMU::~VirtualIMU()
{

}

bool VirtualIMU::GetIMUData( const imu_pack_dsp & imuData )
{   
   dataArray.check_push( imuData );
   return true;
}

bool VirtualIMU::OutputData( imu_pack_dsp & imuData )
{
   bool result = dataArray.try_pop(imuData);
   return result;
}

ImuClient::~ImuClient()
{
}

bool ImuClient::InitMmap()
{
   return true;
}

bool ImuClient::GetImuData( struct imu_pack_dsp* dataArray,
                            int32_t        max_count,
                            int32_t*       returned_sample_count )
{
   size_t i;
   for( i = 0; i < max_count; i++ )
   {
      if( !VirtualIMU::OutputData( dataArray[i] ) )
      {
         break;
      }
   }
   *returned_sample_count = i;
   return true;
}

bool ImuClient::ConnectServer()
{
   return true;
}

bool ImuClient::SendMsgStart( int sensor )
{
   return true;
}

bool ImuClient::SendMsgStop( int sensor )
{
   return true;
}

bool ImuClient::SendMsgConfigRate( int rate )
{
   return true;
}

bool ImuClient::SendMsgConfigDataType( int type )
{
   return true;
}
