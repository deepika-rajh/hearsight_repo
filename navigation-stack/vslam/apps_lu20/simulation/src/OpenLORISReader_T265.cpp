/*******************************************************************************
@copyright
Copyright (c) 2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include "OpenLORISReader_T265.h"

void ReadMatrix( std::ifstream & file, float * matrix );


#include "opencv2/opencv.hpp"
#include <fstream>

void copyToMat( const rvPose6DRT & s, cv::Mat &d );

OpenLORISReaderT265::OpenLORISReaderT265( const std::string & imageListFile ):
   OpenLORISReader( imageListFile )
{
   ReadImageList( imageListFile, "fisheye1.txt", "fisheye2.txt", imageList );
   ReadCameraConfig( imageListFile+"sensors.yaml", cameraConfig );
   ReadExtrinsicParams( imageListFile + "trans_matrix.yaml", 
                        "t265_fisheye1_optical_frame", "t265_fisheye2_optical_frame", "t265_accelerometer",
                        imuCamera, cameraBaselink, cameraLCameraR );
   cv::Mat r( cv::Size(3, 4), CV_32FC1 ), rInv;
   copyToMat( cameraLCameraR, r );
   rInv = r.inv();
   cameraConfig.translation[0] = rInv.at<float>( 0, 3 );
   cameraConfig.translation[1] = rInv.at<float>( 1, 3 );
   cameraConfig.translation[2] = rInv.at<float>( 2, 3 );
   cv::Mat rv( cv::Size( 1, 3 ), CV_32FC1 );
   r = rInv.colRange( 0, 3 ).rowRange( 0, 3 );
   cv::Rodrigues( r, rv );
   cameraConfig.rotation[0] = rv.at<float>( 0 );
   cameraConfig.rotation[1] = rv.at<float>( 1 );
   cameraConfig.rotation[2] = rv.at<float>( 2 );
   ReadIMUSamples( imageListFile, "t265_accelerometer.txt", "t265_gyroscope.txt", imuSamples );
   ReadOdomSamples( imageListFile, wheelSamples );
}

OpenLORISReaderT265::~OpenLORISReaderT265()
{

}

bool OpenLORISReaderT265::GetNextFrame( mvFrame & frame, std::vector<sensor_wheel> & wheelOdomSet, std::vector<imu_pack_dsp> & imuSampleSet, std::vector<sensor_hijack> & hijackSet, std::vector<StampedSystemCallback> & callbackSet, std::vector<rvPose6DRTWithTimestamp>& poses)
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
      AllocateMvImage( frame.leftImage, iImage.cols, image.rows, iImage.rows * iImage.step[0] );
   }
   memcpy( frame.leftImage->pixels, iImage.data, iImage.rows * iImage.cols );

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
   if( frame.rightImage->height != depthImage.rows
       || frame.rightImage->width != depthImage.cols
       || frame.rightImage->pixels == NULL )
   {
      ReleaseMVImage( frame.rightImage );
      AllocateMvImage( frame.rightImage, iImage.cols, iImage.rows, depthImage.step[0] );
   }
   memcpy( frame.rightImage->pixels, depthImage.data, depthImage.step[0] * depthImage.rows);
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

void OpenLORISReaderT265::ReadCameraConfig( const std::string & filename, rvStereoCamera & cameraConfig )
{
   ReadMonoCameraConfig( filename, "t265_fisheye1_optical_frame", cameraConfig.camera[0] );
   ReadMonoCameraConfig( filename, "t265_fisheye2_optical_frame", cameraConfig.camera[1] );
}

bool OpenLORISReaderT265::getCameraConfiguration( rvCameraParams & config )
{
   config.imageFormat = Y_ONLY_FORMAT;
   config.cameraType = rvStereo;
   config.stereo = cameraConfig;
   //config.stereoRect.camera[0].initialized = false;
   //config.stereoRect.camera[1].initialized = false;

   //https://github.com/IntelRealSense/librealsense/pull/3951/files
   //We need to determine what focal length our undistorted images should have
   //in order to set up the camera matrices for initUndistortRectifyMap.We
   //could use stereoRectify, but here we show how to derive these projection
   //matrices from the calibration and a desired height and field of view

   //We calculate the undistorted focal length :
   //
   //         h
   // -----------------
   //  \      |      /
   //    \    | f  /
   //     \   |   /
   //      \ fov /
   //        \|/
   float stereo_fov_rad = 100 * (3.14159f / 180);  // 100 degree desired fov
   int stereo_height_px = 480;                   // 300x300 pixel stereo output
   float stereo_focal_px = stereo_height_px / 2 / tan( stereo_fov_rad / 2 );

   //We set the left rotation to identity and the right rotation
   //the rotation between the cameras
   //cv::Mat  R_left = cv::Mat::eye( 3, 3, CV_32FC1 );
   cv::Mat  R_right;
   cv::Mat r( 3, 1, CV_32FC1 );
   memcpy( r.data, cameraConfig.rotation, sizeof( cameraConfig.rotation ) );
   cv::Rodrigues( -r, R_right );  //if right's attitude is identity, left's attitude is r. if left is idenetiy, right's is -r.

   //The stereo algorithm needs max_disp extra pixels in order to produce valid
   //disparity on the desired output region.This changes the width, but the
   //center of projection should be on the center of the cropped image
   float max_disp = 128; //16*8
   int stereo_width_px = stereo_height_px + max_disp;
   //float stereo_size = (stereo_width_px, stereo_height_px);
   float stereo_cx = (stereo_height_px - 1) / 2 + max_disp;
   float stereo_cy = (stereo_height_px - 1) / 2;

   //Construct the left and right projection matrices, the only difference is
   // that the right projection matrix should have a shift along the x axis of
   // baseline*focal_length
   //cv::Mat P_left = cv::Mat::zeros( 3, 4, CV_32FC1 );
   //P_left.at<float>( 0, 0 ) = stereo_focal_px;
   //P_left.at<float>( 0, 2 ) = stereo_cx;
   //P_left.at<float>( 1, 1 ) = stereo_focal_px;
   //P_left.at<float>( 1, 2 ) = stereo_cy;
   //P_left.at<float>( 2, 2 ) = 1.f;
   //cv::Mat P_right = P_left.clone();
   //P_right.at<float>( 0, 3 ) = cameraConfig.translation[0] * stereo_focal_px;

   config.stereoRect.camera[0].initialized = true;
   config.stereoRect.camera[0].pixelHeight = stereo_height_px;
   config.stereoRect.camera[0].pixelWidth = stereo_width_px;
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
   config.stereoRect.camera[0].P[0][0] = stereo_focal_px;
   config.stereoRect.camera[0].P[0][2] = stereo_cx;
   config.stereoRect.camera[0].P[1][1] = stereo_focal_px;
   config.stereoRect.camera[0].P[1][2] = stereo_cy;
   config.stereoRect.camera[0].P[2][2] = 1.f;

   config.stereoRect.camera[1] = config.stereoRect.camera[0];
   for( size_t i = 0; i < 3; i++ )
   {
      config.stereoRect.translation[i] = cameraConfig.translation[i];
      for( size_t j = 0; j < 3; j++ )
      {
         config.stereoRect.camera[1].R[i][j] = R_right.at<float>( i, j );
      }
   }
   config.stereoRect.camera[1].P[0][3] = cameraConfig.translation[0] * stereo_focal_px;

   return true;
}


