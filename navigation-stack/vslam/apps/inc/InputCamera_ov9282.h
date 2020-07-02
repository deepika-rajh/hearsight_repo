/*****************************************************************************
@copyright
Copyright (c) 2020 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef _INPUT_CAMERA_OV9282_H_
#define _INPUT_CAMERA_OV9282_H_

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

#include "rvCamera.h"

#define QMMFSRC    "qtiqmmfsrc"

class InputCamera_OV9282
{
public:

   typedef void( *CameraCallback )(const int64_t, const unsigned char *, const unsigned short *);

   InputCamera_OV9282(const char * configureFile);
   ~InputCamera_OV9282();

   InputCamera_OV9282( const InputCamera_OV9282 & ) = delete;

   bool start();
   bool stop();

   void addCallback( CameraCallback callback );
   //void onPreviewFrame( ICameraFrame *frame );

   const rvCameraParams & getCameraConfiguration( ) const;
   bool ParsePlaybackParameters( const char *configFile );

protected:
   std::shared_ptr<std::thread> thread;
   void proc();

private:
   bool running;
   CameraCallback callback;

   void findClocksOffsetForCamera();
   bool setup_pipeline();
   static gboolean bus_callback (GstBus *bus, GstMessage *msg, gpointer userData);

   rvCameraParams configuration;

   //time stamp
   int64_t realClock;
   int64_t monotonicClock;
   int64_t clockOffset;

   GstElement *gst_pipeline;
   GstElement *gst_src;
   GstElement *gst_filter;
   GstElement *gst_sink;
   
   #ifdef ROS_BASED
   int64_t time_offset;
   #endif
};
#endif
