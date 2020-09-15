/*****************************************************************************
@copyright
Copyright (c) 2020 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef _INPUT_CAMERA_OV9282_H_
#define _INPUT_CAMERA_OV9282_H_

#include <mutex>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>

#include "rvCamera.h"

#define QMMFSRC    "qtiqmmfsrc"
typedef void( *CameraCallback )(const int64_t, const unsigned char *, const unsigned short *);

class InputCamera_OV9282
{
public:

   InputCamera_OV9282(const char * configureFile);
   ~InputCamera_OV9282();

   InputCamera_OV9282( const InputCamera_OV9282 & ) = delete;

   bool start();
   bool stop();

   void addCallback( CameraCallback callback );

   const rvCameraParams & getCameraConfiguration( ) const;
   bool ParsePlaybackParameters( const char *configFile );

protected:
   std::shared_ptr<std::thread> rawImgThread;
   void getRawImgProc();

private:
   bool running;
   static CameraCallback callback;

   void findClocksOffsetForCamera();
   bool setup_pipeline();
   static void eos_callback (GstBus *bus, GstMessage *msg, gpointer userData);
   static void error_callback (GstBus *bus, GstMessage *msg, gpointer userData);
   static void state_change_cb (GstBus *bus, GstMessage *msg, gpointer userData);
   static GstFlowReturn new_sample_cb (GstElement *sink, gpointer userdata);

   rvCameraParams configuration;

   //time stamp
   int64_t realClock;
   int64_t monotonicClock;
   int64_t clockOffset;

   GstElement *gst_pipeline;
   GMainLoop  *gst_loop;
   
   static int64_t time_offset;
};
#endif
