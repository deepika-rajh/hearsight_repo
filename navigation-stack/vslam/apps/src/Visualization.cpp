/*****************************************************************************
@copyright
Copyright (c) 2020 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <functional>
#include <string.h>
#include "SystemTime.h"
#include "Visualization.h"
#include "VSLAMSystem.h"

#ifdef ROS_BASED
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#endif

Visualiser::Visualiser( int width, int height )
{
   imageHeight = height;
   imageWidth = width;
   undistortedImage = new uint8_t[imageHeight*imageWidth];

   gridHeight = 0;
   gridWidth = 0;
   gridImage = NULL;

   occupancyGridHeight = 0;
   occupancyGridWidth = 0;
   occupancyGridImage = NULL;
}

Visualiser::~Visualiser()
{
   delete[] undistortedImage;

   if( gridImage != NULL )
      delete[]gridImage;

   occupancyGridMutex.lock();
   if( occupancyGridImage != NULL )
   {
      WriteGrayBitmap( occupancyGridImage, "OccupancyImage.bmp", occupancyGridWidth, occupancyGridHeight, 0, 0, occupancyGridWidth, 0 );
      delete[]occupancyGridImage;
   }
      
   occupancyGridMutex.unlock();
}


void 
Visualiser::ShowPoints(RV_VSLAM_TRACKING_STATE quality, std::string title, const rvVWSLAMStatus &status )
{   
#ifdef OPENCV_ENABLED
   cv::Mat rview;
   DrawLabelledImage( quality, undistortedImage, imageWidth, imageHeight, status, rview );

   //char imageName[256];
   //sprintf( imageName, "%s_labelledImage_%" PRId64 ".png", title.c_str(), poseWithTime.timestamp );
   //cv::imwrite( imageName, rview );
#endif

//#ifndef ARM_BASED
#ifndef __linux__
   cv::imshow( title, rview );
   cv::waitKey( 1 );
#endif

#ifdef ROS_BASED
   extern image_transport::Publisher    labeled_img_pub;
   static int divider = 0;

   if(divider == 0)
   {
       sensor_msgs::msg::Image::SharedPtr img;
       img = cv_bridge::CvImage(
       std_msgs::msg::Header(), sensor_msgs::image_encodings::BGR8, rview).toImageMsg();

       rclcpp::Clock ros_clock(RCL_ROS_TIME);

       img->width = imageWidth;
       img->height = imageHeight;
       img->is_bigendian = false;
       img->step = imageWidth * 3;
       img->header.frame_id = "labeled_image";
       img->header.stamp = ros_clock.now();
       labeled_img_pub.publish(img);
   }

   divider++;
   if(divider > 14) divider = 0;
#endif

   return;
}

void
Visualiser::ShowGridMap()
{
   if( gridHeight <= 0 || gridWidth <= 0 || gridImage == NULL )
   {
      return;
   }

#ifdef OPENCV_ENABLED
   cv::Mat rview( gridHeight, gridWidth, CV_8UC1 );
   memcpy( rview.data, gridImage, gridHeight*gridWidth );
   #ifndef __linux__
      cv::imshow( "grid map", rview );
      cv::waitKey( 1 );
   #endif
#endif

#ifdef ROS_BASED
   extern image_transport::Publisher    occupancy_img_pub;
   static int divider = 0;

   if( divider == 0 )
   {
      cv::Mat rview = cv::Mat( gridHeight, gridWidth, CV_8UC1 );
      memcpy( rview.data, gridImage, gridHeight* gridWidth );
      cv::cvtColor( rview, rview, cv::COLOR_GRAY2BGR ); //TODO: not sure if RGB is a MUST

      sensor_msgs::msg::Image::SharedPtr img;
      img = cv_bridge::CvImage(
         std_msgs::msg::Header(), sensor_msgs::image_encodings::BGR8, rview ).toImageMsg();

      rclcpp::Clock ros_clock( RCL_ROS_TIME );

      img->width = gridWidth;
      img->height = gridHeight;
      img->is_bigendian = false;
      img->step = gridWidth * 3;
      img->header.frame_id = "occupancy_image";
      img->header.stamp = ros_clock.now();
      occupancy_img_pub.publish( img );
   }

   divider++;
   if( divider > 14 ) divider = 0;
#endif

   occupancyGridMutex.lock(); // just lock here in case the data might be visited outside
   if( occupancyGridHeight*occupancyGridWidth < gridWidth*gridHeight )
   {
      if( occupancyGridImage != NULL )
         delete[]occupancyGridImage;
      occupancyGridImage = new unsigned char[gridWidth*gridHeight];
   }
   occupancyGridHeight = gridHeight;
   occupancyGridWidth = gridWidth;
   memcpy( occupancyGridImage, gridImage, gridWidth*gridHeight );
   occupancyGridMutex.unlock();

   // we always release the memory here. Please allocate another memory to hold if necessary
   if( gridImage != NULL )
      delete[]gridImage;
   gridImage = NULL;

   return;
}

#ifdef OPENCV_ENABLED
void Visualiser::DrawLabelledImage(RV_VSLAM_TRACKING_STATE quality, const uint8_t * image, int widthFrame, int heightFrame, const rvVWSLAMStatus & status, cv::Mat & rview )
{
   rview = cv::Mat( heightFrame, widthFrame, CV_8UC1 );
   memcpy( rview.data, image, heightFrame* widthFrame );

   cv::cvtColor( rview, rview,cv::COLOR_GRAY2BGR );
   
   int obsNum = status._MatchedMapPointNum + status._MisMatchedMapPointNum;
   if( obsNum > 0 )
   {
      if( quality != RV_VSLAM_TRACKING_STATE::RV_VSLAM_TRACKING_STATE_FAILED)
      {
         cv::Point2f imagePoint;
         for( int i = 0; i < obsNum; i++ )
         {
            imagePoint.x = status.observationBuf[i].x;
            imagePoint.y = status.observationBuf[i].y;
            if( status.observationBuf[i].s == RV_TrackedObservation::RV_OBSERVATION_STATE::MATCHING_OK)
            {
               circle( rview, imagePoint, 4, cv::Scalar( 0, 255, 0 ) ); //green: good feature
            }
            else
            {
               circle( rview, imagePoint, 4, cv::Scalar( 0, 0, 255 ) ); //red: bad feature
            }
         }
      }
   }
   
   char strFrame[60];
   cv::Scalar color( 255, 0, 0 );
   if(RV_VSLAM_TRACKING_STATE::RV_VSLAM_TRACKING_STATE_FAILED == quality ||
	   RV_VSLAM_TRACKING_STATE::RV_VSLAM_TRACKING_STATE_INITIALIZING == quality
       || RV_VSLAM_TRACKING_STATE::RV_VSLAM_TRACKING_STATE_SCALEESTIMATION == quality
       )
   {
      color = cv::Scalar( 0, 0, 255 ); //red
   }
   else if( quality == RV_VSLAM_TRACKING_STATE::RV_VSLAM_TRACKING_STATE_BAD)
   {
      color = cv::Scalar( 0, 255, 255 );
   }

   static int frameIndex = 0;
   frameIndex++;

   snprintf( strFrame, 30, "BR:%3d, KF:%5d", (int32_t)status._Brightness, status._KeyframeNum );
   //putText( rview, std::string( strFrame ), cv::Point2f( widthFrame - 290.0f, heightFrame - 30.0f ), cv::FONT_HERSHEY_COMPLEX, 0.6, color );
   putText( rview, std::string( strFrame ), cv::Point2f( widthFrame - 290.0f, heightFrame - 30.0f ), cv::FONT_HERSHEY_COMPLEX, 0.6, color );
   snprintf( strFrame, 60, "FrameIndex = %4d, Mismatched: %3d, Matched: %3d", frameIndex, status._MisMatchedMapPointNum, status._MatchedMapPointNum );
   putText( rview, std::string( strFrame ), cv::Point2f( widthFrame - 600.0f, 30.0f ), cv::FONT_HERSHEY_COMPLEX, 0.6, color );

   return;
}
#endif


void Visualiser::WriteGrayBitmap( unsigned char *iImgData, char *iImgName, int iWidth, int iHeight, int iPosX, int iPosY, int iFullLine, int Flag )
{
   int i, column, iNewWidth, iNewHeight;
   unsigned short pp;
   unsigned int pp1;
   unsigned char CenterValue;
   int pp2;
   FILE *file;
   iNewWidth = iWidth;
   iNewHeight = iHeight;
   i = iNewWidth % 4 == 0 ? iNewWidth : (4 * (iNewWidth / 4 + 1));
   file = fopen( iImgName, "wb" );
   pp = 0x4d42;
   fwrite( &pp, 2, 1, file );
   pp1 = i*iNewHeight + 1078;
   fwrite( &pp1, 4, 1, file );
   fwrite( &pp, 2, 1, file );
   fwrite( &pp, 2, 1, file );
   pp1 = 1078;
   fwrite( &pp1, 4, 1, file );

   pp1 = 40;
   fwrite( &pp1, 4, 1, file );
   pp2 = iNewWidth;
   fwrite( &pp2, 4, 1, file );
   pp2 = iNewHeight;
   fwrite( &pp2, 4, 1, file );
   pp = 1;
   fwrite( &pp, 2, 1, file );
   pp = 8;
   fwrite( &pp, 2, 1, file );
   pp1 = 0;
   fwrite( &pp1, 4, 1, file );
   pp1 = iNewHeight*i;
   fwrite( &pp1, 4, 1, file );
   pp2 = 0;
   fwrite( &pp2, 4, 1, file );
   fwrite( &pp2, 4, 1, file );
   pp1 = 0;
   fwrite( &pp1, 4, 1, file );
   fwrite( &pp1, 4, 1, file );
   for( pp2 = 0; pp2<256; ++pp2 )
   {
      CenterValue = pp2;
      fwrite( &CenterValue, 1, 1, file );
      fwrite( &CenterValue, 1, 1, file );
      fwrite( &CenterValue, 1, 1, file );
      fwrite( &CenterValue, 1, 1, file );
   }

   if( Flag )
      column = iFullLine % 4 == 0 ? iFullLine : (4 * (iFullLine / 4 + 1));
   else
      column = iFullLine;
   for( pp2 = 0; pp2<iNewHeight; ++pp2 )
   {
      fwrite( iImgData + (iPosY + iNewHeight - 1 - pp2)*column + iPosX, 1, iNewWidth, file );
      if( i>iNewWidth )
         fwrite( &column, 1, i - iNewWidth, file );
   }

   fclose( file );
}
