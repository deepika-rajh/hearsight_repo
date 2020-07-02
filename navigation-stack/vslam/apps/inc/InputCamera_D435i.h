/*****************************************************************************
@copyright
Copyright (c) 2020 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef _INPUT_CAMERA_D435I_H_
#define _INPUT_CAMERA_D435I_H_

#include "rvCamera.h"
#include <librealsense2/rs.hpp>

class InputCamera_D435i
{
public:

   typedef void( *CameraCallback )(const int64_t, const unsigned char *, const unsigned short *);

   InputCamera_D435i();
   ~InputCamera_D435i();

   InputCamera_D435i( const InputCamera_D435i & ) = delete;

   bool start();
   bool stop();

   void addCallback( CameraCallback callback );

   const rvCameraParams & getCameraConfiguration( ) const;

protected:
   std::shared_ptr<std::thread> thread;
   void proc();

private:
   bool running;
   CameraCallback callback;

   void findClocksOffsetForCamera();

   //time stamp
   int64_t realClock;
   int64_t monotonicClock;
   int64_t clockOffset;

   rs2::pipeline pipe;
   rs2::pipeline_profile profiles;
   rs2::config cfg;
   rs2_intrinsics intrinsics;
   rvCameraParams configuration;

};
#endif
