/*******************************************************************************
@copyright
Copyright (c) 2017-2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include "PoseReader.h"
#include <string>
#include "opencv2/opencv.hpp"


PoseReader::PoseReader( const std::string & callbackName )
{
   poseStream.open( callbackName.c_str() );
   if( !poseStream.is_open() )
   {
      printf( "Cannot open file %s for pose reading!\n", callbackName.c_str() );
   }
   else
   {
      printf( "Open file %s for pose reading!\n", callbackName.c_str() );
      //fileReady = true;
   }
   curPose.timestamp = 0;
}

PoseReader::~PoseReader()
{

}


bool PoseReader::getPose( uint64_t timestamp, rvPose6DRTWithTimestamp& pose )
{
   bool result = true;
   if( curPose.timestamp == 0 )
   {
      result = getPose( curPose );
   }
   
   if( curPose.timestamp < timestamp && result )
   {
      pose = curPose;
      curPose.timestamp = 0;
   }
   else
   {
      pose.timestamp = 0;
   }

   return result;
}

bool PoseReader::getPose(rvPose6DRTWithTimestamp& pose )
{
   if( !poseStream.is_open() || poseStream.eof() )
   {
      return false;
   }

   cv::Mat r(1, 3, CV_32FC1);
   float32_t tx, ty, tz;
   std::string buffer;

   std::getline(poseStream, buffer, ',');
   pose.timestamp = std::stoll(buffer);

   std::getline(poseStream, buffer, ',');
   tx = std::stof(buffer);
   std::getline(poseStream, buffer, ',');
   ty = std::stof(buffer);
   std::getline(poseStream, buffer, ',');
   tz = std::stof(buffer);
   std::getline(poseStream, buffer, ',');
   r.at<float>(0) = std::stof(buffer);
   std::getline(poseStream, buffer, ',');
   r.at<float>(1) = std::stof(buffer);
   std::getline(poseStream, buffer);
   r.at<float>(2) = std::stof(buffer);

   cv::Mat R;
   cv::Rodrigues(r, R);
   for (size_t i = 0; i < 3; i++)
      for (size_t j = 0; j < 3; j++)
         pose.pose.matrix[i][j] = R.at<float>(i, j);

   pose.pose.matrix[0][3] = tx;
   pose.pose.matrix[1][3] = ty;
   pose.pose.matrix[2][3] = tz;

   return true;
}


