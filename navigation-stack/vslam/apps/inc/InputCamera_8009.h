/*****************************************************************************
@copyright
Copyright (c) 2020 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef _INPUT_CAMERA_8009_H_
#define _INPUT_CAMERA_8009_H_

#include "VSLAMCamera.h"

/*    real camera       */
#ifndef ARM_BASED 
#include "camera_parameters_win.h"
#else
#include "camera_parameters.h"
#endif
#include <camera.h>

using namespace camera;

struct CameraCapabilities
{
   std::vector<ImageSize> pSizes, vSizes;
   std::vector<std::string> focusModes, wbModes, isoModes;
   Range brightness, sharpness, contrast;
   std::vector<Range> previewFpsRanges;
   std::vector<VideoFPS> videoFpsValues;
};

#define MAX_NUM_CAMERAS 2
#define DOWN_CAMERA_BLUR_IDX 0
#define FRONT_CAMERA_BLUR_IDX 1


#define EAGLE_DEFAULT_EXPOSURE_VALUE_STR "250"

#define EAGLE_MIN_EXPOSURE_VALUE 1
// Row period (It was 19.333 us before, current version exacted value is unknown, but around 19.333)
// Exposure time = RowPeriod * ExposureValue
// Exposure time should less than interval time between frames
// FrameLength = (1e6/ frame rate / RowPeriod)
// FrameLength is set at 1724 for 30fps camera (QCT value)
// MAX_EXPOSURE_VALUE = FrameLength - OffSet
#define FRAMELENGTH_30FPS   1724
#define EAGLE_OFFSET_EXPOSURE_VALUE 20


//Real gain is between 1.0 to 16.0
//But the captured image is too noisy if real gain > 4.0
//Input value is real gain * 256
#define EAGLE_DEFAULT_GAIN_VALUE_STR "0"
#define EAGLE_MIN_GAIN_VALUE 256    
#define EAGLE_MAX_GAIN_VALUE 1024   

class InputCamera_8009 : ICameraListener
{
public:

   typedef void( *CameraCallback )(const int64_t, const unsigned char *, const unsigned short *);

   InputCamera_8009();
   ~InputCamera_8009();

   InputCamera_8009( const InputCamera_8009 & ) = delete;

   void setCaptureParams( const VSLAMCameraParams& params );

   void setExposureAndGain( float32_t exposure, float32_t gain, int &exposureValue, int &gainValue );

   bool init();
   bool deinit();

   bool start();
   bool stop();

   void addCallback( CameraCallback callback );

   /* listener methods */
   virtual void onError();
   virtual void onPreviewFrame( ICameraFrame* frame );
   virtual void onVideoFrame( ICameraFrame* frame );
   void printCapabilities();
   void findClocksOffsetForCamera();

   VSLAMCameraParams cameraParams;

private:
   bool running;
   CameraCallback callback;

   int initialize( int camId );
   int setParameters();

   CameraParams atlParams;
   CameraCapabilities cameraCaps;
   ICameraDevice* camera_;
   int camId;

   //time stamp
   int64_t realClock;
   int64_t monotonicClock;
   int64_t clockOffset;

   mvCPA* cpa;
   void CallCPA();

   uint32_t maxExposureValue;

};
#endif
