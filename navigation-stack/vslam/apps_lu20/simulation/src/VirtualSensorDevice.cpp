/***************************************************************************//**
@copyright
Copyright (c) 2017-2023 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include "VirtualSensorDevice.h"
#include "VirtualHijack.h"
#include "VirtualIMU.h"


#include "ImageListReader.h"
#include "MVReader.h"
#include "TUMRGBDReader.h"
#include "EurocReader.h"
#include "OpenLORISReader_D435i.h"
#include "OpenLORISReader_T265.h"

#include <opencv2/opencv.hpp>

#include <camera.h>
#include "mv.h"

#include <fstream>
#include <functional>

#ifdef WIN32
#include <windows.h>
inline void mySleep(int x)
{
    Sleep(x);
}
#define VSLAM_SLEEP(x)  mySleep(x)
#else
#include <unistd.h>
#define VSLAM_SLEEP(x)  usleep(x*1000)
#endif //WIN32

//extern std::string Program_Root; // Root path for files;
extern std::mutex gMapSavingMutex;

using namespace camera;


RV_API void rvInitPose6DRT(rvPose6DRT* pose)
{
    memset(pose->matrix[0], 0, sizeof(pose->matrix));
    for (size_t i = 0; i < 3; i++)
        pose->matrix[i][i] = 1.0f;
}

RV_API void rvInitWheelConf(rvWheelConfiguration* wheelConf)
{
    wheelConf->wheelEnabled = false;
    rvInitPose6DRT(&wheelConf->baselinkInCamera);
}


RV_API void rvInitIMUConf(rvIMUConfiguration* imuConf)
{
    imuConf->imuEnabled = false;
    memset(imuConf->acceBias, 0, sizeof(imuConf->acceBias));
    memset(imuConf->gyroBias, 0, sizeof(imuConf->gyroBias));
    imuConf->deltaInSecond = 0.f;
    rvInitPose6DRT(&imuConf->cameraInIMU);
}

VirtualSensorDevice::VirtualSensorDevice( const char * sensorSetting, const char * configureFile, int startFrame ):
    cameraCallback(NULL), stateCallback(NULL), wheelCallback(NULL), poseCallback(NULL), isSystemWorking(NULL), waitForRawPose(NULL)
{
   dataReader = NULL;
   Program_Root = sensorSetting;
   rvInitWheelConf( &wheelConfig );
   rvInitIMUConf( &imuConfig );
   std::string sensorType( configureFile );
   ParsePlaybackParameters( sensorType );

   timeOffset = 20000000; //20ms

   imageFrame.rightImage = new mvImage;
   imageFrame.rightImage->pixels = NULL;
   imageFrame.leftImage = new mvImage;
   imageFrame.leftImage->pixels = NULL;
   imageFrame.depthImage = new mvImage16;
   imageFrame.depthImage->pixels = NULL;

   startIndex = 0;
   sequence = startFrame;
   Program_Root = sensorSetting;
}

void VirtualSensorDevice::addCallback(CameraCallback callBack)
{
    cameraCallback = callBack;
}

void VirtualSensorDevice::addStateCallback(StateCallback callBack)
{
    stateCallback = callBack;
}


void VirtualSensorDevice::addWheelCallback(WheelCallback callBack)
{
   wheelCallback = callBack;
}


void VirtualSensorDevice::addPoseCallback(PoseCallback callBack)
{
   poseCallback = callBack;
}


VirtualSensorDevice::~VirtualSensorDevice()
{
   cameraMutex.lock();

   if (imageFrame.leftImage != NULL)
   {
      if (imageFrame.leftImage->pixels != NULL)
         delete[]imageFrame.leftImage->pixels;
      delete imageFrame.leftImage;
   }

   if (imageFrame.rightImage != NULL)
   {
      if (imageFrame.rightImage->pixels != NULL)
         delete[]imageFrame.rightImage->pixels;
      delete imageFrame.rightImage;
   }

   if (imageFrame.depthImage != NULL)
   {
      if (imageFrame.depthImage->pixels != NULL)
         delete[]imageFrame.depthImage->pixels;
      delete imageFrame.depthImage;
   }

   cameraMutex.unlock();
}


bool VirtualSensorDevice::start()
{
   cameraThread = std::make_shared<std::thread>(std::mem_fn(&VirtualSensorDevice::virtualSensorDeviceProc), this);
   return true;
}

bool VirtualSensorDevice::stop()
{
   stopNow = true;
   if (cameraThread)
   {
      cameraThread->join();
   }
   return true;
}

bool VirtualSensorDevice::ParsePlaybackParameters(const std::string& sensorType)
{

   std::string sequenceName;
   cameraConfig.stereo.camera[0].pixelHeight = cameraConfig.stereo.camera[0].pixelWidth = 0;

   dataReader = NULL;
   if (sensorType.compare("OpenLORIS_RGBD") == 0
      || sensorType.compare("OpenLORIS_RGBDW") == 0)
   {
      dataReader = new OpenLORISReaderD435i(Program_Root);

   }
   else if (sensorType.compare("OpenLORIS_Stereo") == 0
      || sensorType.compare("OpenLORIS_StereoW") == 0)
   {
      dataReader = new OpenLORISReaderT265(Program_Root);

   }
   else if (sensorType.compare("RV_RGBD") == 0
      || sensorType.compare("RV_MonoW") == 0
      || sensorType.compare("RV_StereoW") == 0
      || sensorType.compare("RV_RGBDW") == 0
      || sensorType.compare("RV_Stereo") == 0
      )
   {
      dataReader = new MVReader(Program_Root);
   }
   else if (sensorType.compare("EuRoC_Mono") == 0
      || sensorType.compare("EuRoC_MonoI") == 0
      || sensorType.compare("EuRoC_Stereo") == 0
      )
   {
      dataReader = new EurocReader(Program_Root);
   }
   else if (sensorType.compare("TUMRGBD_Mono") == 0
      )
   {
      dataReader = new TUMRGBDReader(Program_Root);
   }
   dataReader->getCameraConfiguration( cameraConfig );
   dataReader->getIMUConfiguration( imuConfig );
   dataReader->getWheelConfiguration( wheelConfig );
   dataReader->getTargetImage(targetImage);

   if( sequenceName.length() > 0 )
   {
#ifdef OPENCV_SUPPORTED
      dataReader = new ImageListReader( sequenceName, wheelodomName, hijackName, std::string() );
#endif //OPENCV_SUPPORTED
   }
   //else if( !mTUMPath.empty() )
   //{
   //   dataReader = new TUMRGBDReader(mTUMPath);
   //   //configuration is mainly read from camera configuration file in virtual sensor device
   //   //Here some other properties are setted in function below, which are not included in the configuration file
   //   dataReader->getCameraConfiguration( cameraConfig );
   //}



   // Select the image format

   if( Y_ONLY_FORMAT == cameraConfig.imageFormat )
   {
      formatVar = 4;
   }
   else if( RAW_FORMAT == cameraConfig.imageFormat)
   {

      formatVar = 5;
   }
   else
   {
      printf( "Only support Y_ONLY_FORMAT and RAW_FORMAT for the simulation!\n" );
      return false;
   }

   if( dataReader == NULL )
   {
      return false;
   }

   return true;
}


const rvCameraParams & VirtualSensorDevice::getCameraConfiguration( ) const
{
   return cameraConfig;
}


const rvIMUConfiguration & VirtualSensorDevice::getIMUConfiguration() const
{
   return imuConfig;
}


const rvWheelConfiguration & VirtualSensorDevice::getWheelConfiguration() const
{
   return wheelConfig;
}


const rvTargetImage& VirtualSensorDevice::getTargetImage() const
{
    return targetImage;
}


void VirtualSensorDevice::virtualSensorDeviceProc()
{
   bool isIMUdataValid = true;
   uint32_t pixelNum;
   uint32_t bufferLength;
   if( cameraConfig.cameraType == rvStereo )
   {
      pixelNum = cameraConfig.stereo.camera[0].pixelWidth * cameraConfig.stereo.camera[0].pixelHeight;
      bufferLength = pixelNum * 2;
   }
   else
   { 
      pixelNum = cameraConfig.stereo.camera[0].pixelWidth * cameraConfig.stereo.camera[0].pixelHeight;
      bufferLength = pixelNum;
   }

   uint8_t * imageBuf = new uint8_t[bufferLength / 4 * formatVar]; //RAW_FORMAT
   memset( imageBuf, 0, bufferLength / 4 * formatVar );

   stopNow = false;

   cameraMutex.lock();
   VSLAM_SLEEP( 500 );
   for(int i=startIndex; true; i++  )
   {
      if( stopNow )
      {
         startIndex = i;
         break;
      }

      //gMapSavingMutex.lock();
      bool result;
      if( i < sequence )
      {
         result = dataReader->SkipNextFrame();
      }
      else
      {
         result = dataReader->GetNextFrame( imageFrame, wheelOdomSet, imuSampleSet, hijackSet, callbackSet, poseSet );
      }
      //gMapSavingMutex.unlock();

      if( !result )
      {
         break;
      }
      if( i < sequence )
      {
         continue;
      }

      uint8_t * srcImage = imageFrame.leftImage->pixels;
      uint8_t * desImage = imageBuf;
      for(uint32_t i= 0; i < pixelNum && srcImage != NULL; i += 4, srcImage += 4, desImage += formatVar )
      {
         memcpy( desImage, srcImage, sizeof( uint8_t ) * 4 );
      }
      if( imageFrame.rightImage )
      {
         srcImage = imageFrame.rightImage->pixels;
         for( uint32_t i = 0; i < pixelNum && srcImage != NULL; i += 4, srcImage += 4, desImage += formatVar )
         {
            memcpy( desImage, srcImage, sizeof( uint8_t ) * 4 );
         }
      }
      for( size_t i = 0; i < wheelOdomSet.size(); i++ )
      {
          if (wheelCallback)
              wheelCallback( &wheelOdomSet[i] );
      }

      for (size_t i = 0; i < poseSet.size(); i++)
      {
          if (poseCallback)
              poseCallback(poseSet[i]);
      }

#ifdef HIJACK_SUPPORTED
      for( size_t i = 0; i < hijackSet.size(); i++ )
      {
         VirtualHijack::GetHijack( hijackSet[i] );
      }
#endif //HIJACKSUPPROTED

#ifdef IMU_SUPPORTED
      for( size_t i = 0; i < imuSampleSet.size(); i++ )
      {
         VirtualIMU::GetIMUData(imuSampleSet[i]);
      }
      VSLAM_SLEEP( 25 );
#endif

      //TODO Xu Lei: now simply use imageFrame.depthImage->pixels!=NULL to justify available depth, do we need a flag, how?
      //VirtualCameraFrame frame( (uint8_t *)imageBuf, imageFrame.timestamp, (uint8_t *)(imageFrame.depthImage->pixels) );
      if (cameraCallback)
          cameraCallback( imageFrame.timestamp, (uint8_t *)imageBuf, imageFrame.depthImage->pixels );

      //TODO: for Yanming
      if (isSystemWorking && waitForRawPose && isSystemWorking())
      {
         waitForRawPose();; // to avoid some frames are not processed        
      }
      else
      {
         VSLAM_SLEEP( 50 ); // sleep to make sure both primary and secondary can access g_ImageBuf          
      }

      for( size_t i = 0; i < callbackSet.size(); i++ )
      {
         std::string command;
         switch( callbackSet[i].callback )
         {
            case KSLEEP:
               command = "sleep";
               break;
            case KAWAKE:
                command = "awake";
               break;
            case KRESET:
               command = "reset";
               break;
            case KSTOP:
                command = "stop";
                break;
            case KNONE:
            default:
               break;
         }
         if (!command.empty())
         {
             if (stateCallback)
                 (*stateCallback)(command);
         }
      }
   }

   delete[] imageBuf;
   if (stateCallback)
       (*stateCallback)("stop");

   cameraMutex.unlock();
   return;
}


