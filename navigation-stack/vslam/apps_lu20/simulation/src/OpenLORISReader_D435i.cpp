/*******************************************************************************
@copyright
Copyright (c) 2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include "OpenLORISReader_D435i.h"

void ReadMatrix( std::ifstream & file, float * matrix );


#include "opencv2/opencv.hpp"
#include <fstream>

void copyToMat( const rvPose6DRT & s, cv::Mat &d );

OpenLORISReaderD435i::OpenLORISReaderD435i( const std::string & imageListFile ):
   OpenLORISReader( imageListFile )
{
   ReadImageList( imageListFile, "color.txt", "aligned_depth.txt", imageList );
   ReadMonoCameraConfig( imageListFile + "sensors.yaml", "d400_color_optical_frame", cameraConfig );
   ReadExtrinsicParams( imageListFile + "trans_matrix.yaml", "d400_color_optical_frame", 
                        "d400_depth_optical_frame", "d400_accelerometer",
                        imuCamera, cameraBaselink, cameraLCameraR );
   ReadIMUSamples( imageListFile, "d400_accelerometer.txt", "d400_gyroscope.txt", imuSamples );
   ReadOdomSamples( imageListFile, wheelSamples );

}

OpenLORISReaderD435i::~OpenLORISReaderD435i()
{

}

bool OpenLORISReaderD435i::GetNextFrame( mvFrame & frame, std::vector<sensor_wheel> & wheelOdomSet, std::vector<imu_pack_dsp> & imuSampleSet, std::vector<sensor_hijack> & hijackSet, std::vector<StampedSystemCallback> & callbackSet, std::vector<rvPose6DRTWithTimestamp>& poses)
{
   //Read image from file
   if( curImageIndex >= imageList.size() )
   {
      return false;
   }
   cv::Mat image = cv::imread( imageList[curImageIndex].grayImageName, cv::IMREAD_UNCHANGED );
   printf( "%s\n", imageList[curImageIndex].grayImageName.c_str() );
   if( nullptr == image.data )
   {
      printf( "Cannot read image with name: %s\n", imageList[curImageIndex].grayImageName.c_str() );
      imageList.resize( curImageIndex );
      return false;
   }

   cv::Mat iImage;
   if( image.channels() != 1 )
   {
      cv::cvtColor( image, iImage, cv::COLOR_BGR2GRAY );
   }
   else
   {
      iImage = image;
   }
   memcpy( frame.cameraName, "test", 5 );

   if( frame.leftImage->height != iImage.rows
       || frame.leftImage->width != iImage.cols
       || frame.leftImage->pixels == NULL )
   {
      ReleaseMVImage( frame.leftImage );
      AllocateMvImage( frame.leftImage, iImage.cols, image.rows, iImage.step[0] );
   }
   memcpy( frame.leftImage->pixels, iImage.data, iImage.rows * iImage.step[0] );

   //Get the timestamp

   cv::Mat depthImage = cv::imread( imageList[curImageIndex].depthImageName, cv::IMREAD_UNCHANGED );
   if( depthImage.channels() != 1 )
   {
      cv::cvtColor( depthImage, iImage, cv::COLOR_BGR2GRAY );
   }
   else
   {
      iImage = depthImage;
   }
   if( frame.depthImage->height != depthImage.rows
       || frame.depthImage->width != depthImage.cols
       || frame.depthImage->pixels == NULL )
   {
      ReleaseMVImage( frame.depthImage );
      AllocateMvImage( frame.depthImage, iImage.cols, iImage.rows );
   }
   memcpy( frame.depthImage->pixels, depthImage.data, depthImage.step[0] * depthImage.rows);
   frame.timestamp = imageList[curImageIndex].timestamp;
   
   curImageIndex++;


   imuSampleSet.clear();
   while ( curIMUIndex < imuSamples.size() && imuSamples[curIMUIndex].time_acc < frame.timestamp + 2e7 )
   {
      imuSampleSet.push_back( imuSamples[curIMUIndex] );
      curIMUIndex++;
   } 

   wheelOdomSet.clear();
   while( curWheelIndex < wheelSamples.size() && wheelSamples[curWheelIndex].timestamp < frame.timestamp + 2e7 )
   {
      wheelOdomSet.push_back( wheelSamples[curWheelIndex] );
      curWheelIndex++;
   }

   return true;

}

bool OpenLORISReaderD435i::getCameraConfiguration( rvCameraParams & config )
{
   config.imageFormat = YUV_FORMAT;
   config.cameraType = rvGrayDepth;
   config.stereo.camera[0] = cameraConfig;
   config.stereoRect.camera[0].initialized = true;
   for( size_t i = 0; i < 3; i++ )
   {
      for( size_t j = 0; j < 3; j++ )
      {
         config.stereoRect.camera[0].P[i][j] = 0;
         config.stereoRect.camera[0].R[i][j] = 0;
      }
      config.stereoRect.camera[0].P[i][3] = 0;
      config.stereoRect.camera[0].R[i][i] = 1;
   }
   config.stereoRect.camera[0].P[0][0] = cameraConfig.focalLength[0];
   config.stereoRect.camera[0].P[1][1] = cameraConfig.focalLength[1];
   config.stereoRect.camera[0].P[0][2] = cameraConfig.principalPoint[0];
   config.stereoRect.camera[0].P[1][2] = cameraConfig.principalPoint[1];
   config.stereoRect.camera[0].pixelHeight = cameraConfig.pixelHeight;
   config.stereoRect.camera[0].pixelWidth = cameraConfig.pixelWidth;

   return true;
}
