/*******************************************************************************
@copyright
Copyright (c) 2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include "ImageListReader.h"

#include "opencv2/opencv.hpp"
#include <fstream>

ImageListReader::ImageListReader( const std::string & imageListFile, const std::string & wheelFile, const std::string & hijackName, const std::string & callbackName ):
   SlamDataReader(wheelFile, hijackName, callbackName,"")
{
   ReadImageList( imageListFile, imageList );
   curImageIndex = 0;
}

ImageListReader::~ImageListReader()
{

}

bool ImageListReader::SkipNextFrame()
{
   return false;
}

bool ImageListReader::GetNextFrame( mvFrame & frame, std::vector<sensor_wheel> & wheelOdomSet, std::vector<imu_pack_dsp> & imuSampleSet, std::vector<sensor_hijack> & hijackSet, std::vector<StampedSystemCallback> & callbackSet, std::vector<rvPose6DRTWithTimestamp>& poses)
{
   imuSampleSet.clear();

   //Read image from file
   if( curImageIndex >= imageList.size() )
   {
      return false;
   }
   cv::Mat image = cv::imread( imageList[curImageIndex] );
   printf( "%s\n", imageList[curImageIndex].c_str() );
   if( nullptr == image.data )
   {
      printf( "Cannot read image with name: %s\n", imageList[curImageIndex].c_str() );
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
      AllocateMvImage( frame.leftImage, iImage.cols, image.rows, iImage.rows * iImage.step );
   }
   memcpy( frame.leftImage->pixels, iImage.data, iImage.rows * iImage.cols );

   //Get the timestamp
   size_t indexDot = imageList[curImageIndex].find_last_of( '.' );
   size_t index = imageList[curImageIndex].find_last_of( '_' );
   std::string stamp = imageList[curImageIndex].substr( index + 1, indexDot - index - 1 );
   
   frame.timestamp = std::stoll( stamp );
   
   curImageIndex++;

   //Get wheel odom
   GetWheelOdom( frame.timestamp + (int64_t)2e7, wheelOdomSet );
   GetHijack( frame.timestamp + (int64_t)2e7, hijackSet );
   return true;

}

void ImageListReader::ReadImageList( const std::string & imageListFile, std::vector<std::string> & imageNameSet )
{
   std::ifstream cfg( imageListFile.c_str(), std::ifstream::in );
   if( !cfg.is_open() )
   {
      printf( "Fail to open image list file!!!\n" );
   }
   printf( "Open image name list file: %s\n", imageListFile.c_str() );

   std::string root = imageListFile;

   // align all path separator into '/'
   static std::string search( "\\" ), replace( "/" );
   for( size_t pos = 0; pos = root.find( search, pos ), pos != std::string::npos; pos += replace.length() )
   {
      root.replace( pos, search.length(), replace );
   }

   std::string temp;
   std::string line;
   size_t index = root.find_last_of( '/' );

   root = root.substr( 0, index + 1 );
   while( std::getline( cfg, line ) )
   {
      if( line.length() == 0 )
      {
         continue;
      }
      if( line[0] == '#' )
      {
         continue;
      }
      temp = root + line;
      imageNameSet.push_back( temp );
   }
   cfg.close();
}

