/*****************************************************************************
@copyright
Copyright (c) 2020-2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#ifndef __STDC_FORMAT_MACROS
#define __STDC_FORMAT_MACROS
#endif
#include <inttypes.h>

#ifdef WIN32
#include <time.h>
#endif

#include "mvVWSLAM_app.h"
#include "Visualization.h"
#include "SystemTime.h"
#include "InputCamera_8009.h"

InputCamera_8009::InputCamera_8009()
{
   running = false;
   callback = NULL;
   camId = 0;
   clockOffset = 0;
   maxExposureValue = FRAMELENGTH_30FPS * 2 - EAGLE_OFFSET_EXPOSURE_VALUE;
   cpa = NULL;

   findClocksOffsetForCamera();
}

InputCamera_8009::~InputCamera_8009()
{
   printf( "release camera!\n" );

   deinit();

   if( cpa != NULL )
   {
      mvCPA_Deinitialize( cpa );
   }
}

void InputCamera_8009::onError()
{
   printf( "camera error!\n" );
}

void InputCamera_8009::findClocksOffsetForCamera()
{
   //int64_t dspClock = (int64_t)getDspClock();
   realClock = (int64_t)getRealTime();
   monotonicClock = getMonotonicTime();
   clockOffset = realClock - monotonicClock;
   //printf( "findClocksOffsetForCamera realClock = %" PRId64 ", monotonicClock=%" PRId64 ", clockOffset=%" PRId64 " \n ", realClock, monotonicClock, clockOffset );
}

//#define PRINT_CLOCKS
void InputCamera_8009::onPreviewFrame( ICameraFrame *frame )
{
   static uint32_t countP = 0;
   //printf( "+++++on preview frame! timestamp=%lld\n", frame->timeStamp );

   static int64_t startTime = 0;
   static int64_t endTime = 0;

   startTime = getRealTime();
   printf("latency deug:: startTime = %" PRId64 ", latency for waiting frame = %" PRId64 "\n", startTime, startTime - endTime);

   if( running == false )
   {
      return;
   }

   if( countP % cameraParams.skipFrame == 0 )
   {
      if( cameraParams.inputFormat == VSLAMCameraParams::RAW_FORMAT )
      {
         //callback( (frame->timeStamp + clockOffset)/1000, frame->data, (uint16_t *)(frame->metadata) );
    	 callback( (frame->timeStamp + clockOffset), frame->data, NULL );
         if( countP % cameraParams.cpaFrameSkip == 0 )
         {
            CallCPA();
         }
      }
      else if( cameraParams.inputFormat == VSLAMCameraParams::YUV_FORMAT )
      {
         callback( (frame->timeStamp + clockOffset), frame->data, NULL );
      }
      else
      {
         printf( "Other image formats except RAW_FORMAT & YUV_FORMAT are not supported!!!!\n " );
      }
   }
   countP++;

   endTime = getRealTime();
   printf("latency debug::vslam latency = %" PRId64 ", endtime = %" PRId64 "\n",endTime - startTime, endTime);
}

void InputCamera_8009::onVideoFrame( ICameraFrame *frame )
{
   printf( "Video mode is not supported!!!\n" );
}

void InputCamera_8009::setCaptureParams( const VSLAMCameraParams & params )
{
   cameraParams = params;
   cameraParams.cpaConfiguration.legacyCost.startExposure = params.exposure;
   cameraParams.cpaConfiguration.legacyCost.startGain = params.gain;

}

int InputCamera_8009::initialize( int camId )
{
   int rc;
   rc = ICameraDevice::createInstance( camId, &camera_ );
   if( rc != 0 )
   {
      printf( "could not open camera %d\n", camId );
      return rc;
   }
   camera_->addListener( this );

   rc = atlParams.init( camera_ );
   if( rc != 0 )
   {
      printf( "failed to init parameters\n" );
      ICameraDevice::deleteInstance( &camera_ );
   }

   return rc;
}

int InputCamera_8009::setParameters()
{
   /* temp: using hard-coded values to test the api
      need to add a user interface or script to get the values to test*/
   //int pSizeIdx = 2;   // 640 , 480
   //int vSizeIdx = 2;   // 640 , 480
   int focusModeIdx = 3;
   int wbModeIdx = 2;
   int isoModeIdx = 5;   /// iso800
   int pFpsIdx = -1;
   int vFpsIdx = -1;
   size_t defaultPFps = 3; /// 30 fps
   size_t defaultVFps = 3; /// 30 fps
   cameraCaps.pSizes = atlParams.getSupportedPreviewSizes();
   cameraCaps.vSizes = atlParams.getSupportedVideoSizes();
   cameraCaps.focusModes = atlParams.getSupportedFocusModes();
   cameraCaps.wbModes = atlParams.getSupportedWhiteBalance();
   cameraCaps.isoModes = atlParams.getSupportedISO();
   cameraCaps.brightness = atlParams.getSupportedBrightness();
   cameraCaps.sharpness = atlParams.getSupportedSharpness();
   cameraCaps.contrast = atlParams.getSupportedContrast();
   cameraCaps.previewFpsRanges = atlParams.getSupportedPreviewFpsRanges();
   cameraCaps.videoFpsValues = atlParams.getSupportedVideoFps();

   ImageSize frameSize;
   frameSize.width = cameraParams.inputPixelWidth;
   frameSize.height = cameraParams.inputPixelHeight;

   if( cameraParams.captureMode == VSLAMCameraParams::PREVIEW )
   {
      printf( "settings preview size %dx%d\n", frameSize.width, frameSize.height );
      atlParams.setPreviewSize( frameSize );
   }
   else if( cameraParams.captureMode == VSLAMCameraParams::VIDEO )
   {
      printf( "settings video size %dx%d\n", frameSize.width, frameSize.height );
      atlParams.setVideoSize( frameSize );
   }

   if( cameraParams.func == VSLAMCameraParams::CAM_FUNC_HIRES )
   {
      printf( "setting ISO mode: %s\n", cameraCaps.isoModes[isoModeIdx].c_str() );
      atlParams.setISO( cameraCaps.isoModes[isoModeIdx] );
      focusModeIdx = cameraCaps.focusModes.size()-1;
      printf( "setting focus mode: %s\n", cameraCaps.focusModes[focusModeIdx].c_str() );
      atlParams.setFocusMode( cameraCaps.focusModes[focusModeIdx] );
      wbModeIdx = cameraCaps.wbModes.size()-1;
      printf( "setting WB mode: %s\n", cameraCaps.wbModes[wbModeIdx].c_str() );
      atlParams.setWhiteBalance( cameraCaps.wbModes[wbModeIdx] );
   }
   else if( cameraParams.func == VSLAMCameraParams::CAM_FUNC_OPTIC_FLOW )
   {
      atlParams.setFocusMode( "infinity" );
   }

   if( cameraCaps.previewFpsRanges.size() <= defaultPFps )
   {
      printf( "default preview fps index %ld greater than number of supported fps ranges %ld \n setting to %ld as default\n", defaultPFps, cameraCaps.previewFpsRanges.size(), cameraCaps.previewFpsRanges.size() - 1 );
      defaultPFps = cameraCaps.previewFpsRanges.size() - 1;
   }
   if( cameraCaps.videoFpsValues.size() <= defaultVFps )
   {
      printf( "default video fps index %ld greater than number of supported fps ranges %ld \n setting to %ld as default\n", defaultVFps, cameraCaps.videoFpsValues.size(), cameraCaps.videoFpsValues.size() - 1 );
      defaultVFps = cameraCaps.videoFpsValues.size() - 1;
   }
   for( size_t i = 0; i < cameraCaps.previewFpsRanges.size(); ++i )
   {
      if ((int)((cameraCaps.previewFpsRanges[i].max) / 1000) == (int)cameraParams.frameRate)
      {
         pFpsIdx = i;
         break;
      }
   }
   for( size_t i = 0; i < cameraCaps.videoFpsValues.size(); ++i )
   {
      if( (int)cameraCaps.videoFpsValues[i] == (int)cameraParams.frameRate )
      {
         vFpsIdx = i;
         break;
      }
   }
   if( pFpsIdx == -1 )
   {
      printf( "couldnt find preview fps index for requested framerate %f setting default index %ld \n", cameraParams.frameRate, defaultPFps );
      pFpsIdx = defaultPFps;
   }
   if( vFpsIdx == -1 )
   {
      printf( "couldnt find video fps index for requested framerate %f setting default index %ld \n", cameraParams.frameRate, defaultVFps );
      vFpsIdx = defaultVFps;
   }

   printf( "setting preview fps range(idx %d): %d, %d\n", pFpsIdx, cameraCaps.previewFpsRanges[pFpsIdx].min,
           cameraCaps.previewFpsRanges[pFpsIdx].max );
   atlParams.setPreviewFpsRange( cameraCaps.previewFpsRanges[pFpsIdx] );
   maxExposureValue = FRAMELENGTH_30FPS * 30 / (cameraCaps.previewFpsRanges[pFpsIdx].max / 1000) - EAGLE_OFFSET_EXPOSURE_VALUE;
   printf( "setting video fps(idx %d): %d\n", vFpsIdx, cameraCaps.videoFpsValues[vFpsIdx] );
   atlParams.setVideoFPS( cameraCaps.videoFpsValues[vFpsIdx] );

   if( cameraParams.inputFormat == VSLAMCameraParams::RAW_FORMAT )
   {
      printf( "setting outputFormat RAW_FORMAT\n" );
      atlParams.set( "preview-format", "bayer-rggb" );
      atlParams.set( "picture-format", "bayer-mipi-10gbrg" );
      atlParams.set( "raw-size", "640x480" );
#ifdef CAMERA_8x96
      atlParams.setPreviewFormat( FORMAT_RAW10 );
#endif
   }
   else if( cameraParams.inputFormat == VSLAMCameraParams::YUV_FORMAT )
   {
      printf( "setting outputFormat YUV_FORMAT\n" );
      //atlParams.setPictureFormat( FORMAT_JPEG );  //it seem no need to set preview format, as default is already yuv
      //atlParams.setPictureSize( picSize_ );
      //atlParams.setPreviewSize( pSize_ ); 
   }
   else if( cameraParams.inputFormat == VSLAMCameraParams::NV12_FORMAT )
   {
      printf( "setting outputFormat nv12\n" );
      atlParams.set( "preview-format", "nv12" );
   }
   printf( "focus mode %s \n", atlParams.getFocusMode().c_str() );
   printf( "set up params \n" );
   int ret = atlParams.commit();
   printf( "set up params done \n" );
   return ret;
}

bool InputCamera_8009::init()
{
   int n = getNumberOfCameras();

   printf( "num_cameras = %d\n", n );

   if( n < 1 )
   {
      printf( "No cameras found.\n" );
      return false;
   }

   camId = -1;

   /* find camera based on function */
   for( int i = 0; i < n; i++ )
   {
      CameraInfo info;
      getCameraInfo( i, info );
      printf( " i = %d , info.func = %d \n", i, info.func );
      if( info.func == cameraParams.func )
      {
         camId = i;
         break;
      }
   }

   if( camId == -1 )
   {
      printf( "Camera not found \n" );
      exit( 1 );
   }

   printf( "initializing camera id=%d\n", camId );

   int ret = initialize( camId );
   if( ret != 0 )
   {
      printf( "ERR: initializing camera with %d id failed with err %d \n", camId, ret );
      return false;
   }

   ret = setParameters();
   if( ret != 0 )
   {
      printf( "ERR: initializing camera with %d id failed with err %d \n", camId, ret );
      return false;
   }

   cameraParams.cpaConfiguration.width = cameraParams.outputPixelWidth;
   cameraParams.cpaConfiguration.height = cameraParams.outputPixelHeight;
   cameraParams.cpaConfiguration.format = MVCPA_FORMAT_GRAY8;

   cpa = mvCPA_Initialize( &cameraParams.cpaConfiguration );
   if( cpa == NULL )
   {
      printf( "ERR: cpa init failed\n" );
   }

   return true;
}

bool InputCamera_8009::deinit()
{
   bool ok = false;
   /* release camera device */
   ICameraDevice::deleteInstance( &camera_ );
   return ok;
}

void InputCamera_8009::printCapabilities()
{
   printf( "Camera capabilities\n" );
   cameraCaps.pSizes = atlParams.getSupportedPreviewSizes();
   cameraCaps.vSizes = atlParams.getSupportedVideoSizes();
   cameraCaps.focusModes = atlParams.getSupportedFocusModes();
   cameraCaps.wbModes = atlParams.getSupportedWhiteBalance();
   cameraCaps.isoModes = atlParams.getSupportedISO();
   cameraCaps.brightness = atlParams.getSupportedBrightness();
   cameraCaps.sharpness = atlParams.getSupportedSharpness();
   cameraCaps.contrast = atlParams.getSupportedContrast();
   cameraCaps.previewFpsRanges = atlParams.getSupportedPreviewFpsRanges();
   cameraCaps.videoFpsValues = atlParams.getSupportedVideoFps();

   printf( "available preview sizes:\n" );
   for( size_t i = 0; i < cameraCaps.pSizes.size(); i++ )
   {
      printf( "%zd: %d x %d\n", i, cameraCaps.pSizes[i].width, cameraCaps.pSizes[i].height );
   }
   printf( "available video sizes:\n" );
   for( size_t i = 0; i < cameraCaps.vSizes.size(); i++ )
   {
      printf( "%zd: %d x %d\n", i, cameraCaps.vSizes[i].width, cameraCaps.vSizes[i].height );
   }
   printf( "available focus modes:\n" );
   for( size_t i = 0; i < cameraCaps.focusModes.size(); i++ )
   {
      printf( "%zd: %s\n", i, cameraCaps.focusModes[i].c_str() );
   }
   printf( "available whitebalance modes:\n" );
   for( size_t i = 0; i < cameraCaps.wbModes.size(); i++ )
   {
      printf( "%ld: %s\n", i, cameraCaps.wbModes[i].c_str() );
   }
   printf( "available ISO modes:\n" );
   for( size_t i = 0; i < cameraCaps.isoModes.size(); i++ )
   {
      printf( "%zd: %s\n", i, cameraCaps.isoModes[i].c_str() );
   }
   printf( "available brightness values:\n" );
   printf( "min=%d, max=%d, step=%d\n", cameraCaps.brightness.min,
           cameraCaps.brightness.max, cameraCaps.brightness.step );
   printf( "available sharpness values:\n" );
   printf( "min=%d, max=%d, step=%d\n", cameraCaps.sharpness.min,
           cameraCaps.sharpness.max, cameraCaps.sharpness.step );
   printf( "available contrast values:\n" );
   printf( "min=%d, max=%d, step=%d\n", cameraCaps.contrast.min,
           cameraCaps.contrast.max, cameraCaps.contrast.step );

   printf( "available preview fps ranges:\n" );
   for( size_t i = 0; i < cameraCaps.previewFpsRanges.size(); i++ )
   {
      printf( "%zd: [%d, %d]\n", i, cameraCaps.previewFpsRanges[i].min,
              cameraCaps.previewFpsRanges[i].max );
   }
   printf( "available video fps values:\n" );
   for( size_t i = 0; i < cameraCaps.videoFpsValues.size(); i++ )
   {
      printf( "%zd: %d\n", i, cameraCaps.videoFpsValues[i] );
   }
}

bool InputCamera_8009::start()
{
   int ret = 0;
   running = true;

   if( !init() )
   {
      printf( "Error in camera.init()!\n" );
      return false;
   }

   if( cameraParams.captureMode == VSLAMCameraParams::PREVIEW )
   {
      printf( "start preview\n" );
      ret = camera_->startPreview();
      if( 0 != ret )
      {
         printf( "ERR: start preview failed %d\n", ret );
      }
   }
   else if( cameraParams.captureMode == VSLAMCameraParams::VIDEO )
   {
      printf( "start recording\n" );
      ret = camera_->startRecording();
      if( 0 != ret )
      {
         printf( "ERR: start recording failed %d\n", ret );
      }
   }

   if( 0 != ret )
      return false;

   printf( "set exposure and gain\n" );
   //Copy values, as setExposureAndGain only updates if different values
   //DK: This is a hack, but should work and not hurt
   float32_t tmpExposure = cameraParams.exposure;
   float32_t tmpGain = cameraParams.gain;
   int exposure;
   int gain;

   cameraParams.exposure = 0.f;
   cameraParams.gain = 0.f;

   setExposureAndGain( tmpExposure, tmpGain, exposure, gain );
   printf( "VSLAMCamera::start() after setExposureAndGain inputExp inputGain setExp setGain = %f %f %d %d\n", tmpExposure, tmpGain, exposure, gain );
   printf( "set exposure and gain finished\n" );
   return true;
}

void InputCamera_8009::setExposureAndGain( float32_t exposure, float32_t gain, int &exposureValue, int &gainValue )
{
   if( exposure == cameraParams.exposure && gain == cameraParams.gain )
   {
      //parameters are the same, don't have to set again
      exposureValue = (int)(EAGLE_MIN_EXPOSURE_VALUE + cameraParams.exposure * (maxExposureValue - EAGLE_MIN_EXPOSURE_VALUE));
      gainValue = (int)(EAGLE_MIN_GAIN_VALUE + cameraParams.gain * (EAGLE_MAX_GAIN_VALUE - EAGLE_MIN_GAIN_VALUE));
      return;
   }

   exposureValue = 0;
   gainValue = 0;

   //printf("setExposureAndGain: %f %f \n", exposure, gain);

   if( exposure >= 0.f && exposure <= 1.f /*&& eagleCaptureParams.func == BlurCameraParams::CAM_FUNC_OPTIC_FLOW*/ )
   {
      cameraParams.exposure = exposure;
      exposureValue = (int)(EAGLE_MIN_EXPOSURE_VALUE + cameraParams.exposure * (maxExposureValue - EAGLE_MIN_EXPOSURE_VALUE));
      char buffer[33];
      //exposureValue = 1000;
      snprintf( buffer, sizeof( buffer ), "%d", exposureValue );
      atlParams.set( "qc-exposure-manual", buffer );
      //printf("Setting exposure value = %d \n", exposureValue);
   }
   if( gain >= 0.f && gain <= 1.f /*&& eagleCaptureParams.func == BlurCameraParams::CAM_FUNC_OPTIC_FLOW*/ )
   {
      cameraParams.gain = gain;
      gainValue = (int)(EAGLE_MIN_GAIN_VALUE + cameraParams.gain * (EAGLE_MAX_GAIN_VALUE - EAGLE_MIN_GAIN_VALUE));
      //gainValue = 500;
      char buffer[33];
      snprintf( buffer, sizeof( buffer ), "%d", gainValue );
      atlParams.set( "qc-gain-manual", buffer );
      //printf("Setting gain value = %d \n", gainValue);
   }
   int result = atlParams.commit();
   //printf( "atlParams.commit() result = %d\n", result );
}

bool InputCamera_8009::stop()
{
   running = false;
   if( cameraParams.captureMode == VSLAMCameraParams::PREVIEW )
   {
      printf( "stop preview\n" );
      camera_->stopPreview();
      printf( "stop preview done\n" );
   }
   else if( cameraParams.captureMode == VSLAMCameraParams::VIDEO )
   {
      printf( "stop recording\n" );
      camera_->stopRecording();
   }
   return true;
}

void InputCamera_8009::addCallback( CameraCallback _callback )
{
   callback = _callback;
}


static int  cpa_count = 0;
void InputCamera_8009::CallCPA()
{
   if( cameraParams.useCPA && cpa )
   {
      float32_t exposure = 0.0f, gain = 0.0f;
      int realExposure = 0, realGain = 0;

      mvCPA_AddFrame( cpa, (uint8_t*)undistortedImage, cameraParams.outputPixelWidth );// config.camera.pixelWidth, config.camera.pixelHeight, config.camera.memoryStride );
      mvCPA_GetValues( cpa, &exposure, &gain );
      
      cpa_count++;

#if 0
      float m = 0.0f;
      uint8_t* data = (uint8_t*)undistortedImage;
      int pix_num = cameraParams.outputPixelWidth * cameraParams.outputPixelHeight;
      for( int i =0; i < pix_num; i++ )
      {
         m += (float) data[i];
      }
      m /= pix_num;
#endif

      if( cpa_count == 3 )
         exposure *= 1.1f;
      if( cameraParams.useCPA && ((cpa_count < 4) || (cpa_count > 10))) //enable new camera parameters
      {
         setExposureAndGain( exposure, gain, realExposure, realGain );
         //printf( "Camera_VSLAM::CallCPA() after setExposureAndGain inputExp inputGain setExp setGain = %f %f %f %f\n", exposure, gain, realExposure, realGain );
      }
      //visualiser->PublishExposureGain( exposure, gain, realExposure, realGain, m );
      if( cpa_count > 100 )
         cpa_count = 100;
   }
}
