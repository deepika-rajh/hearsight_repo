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
}

Visualiser::~Visualiser()
{
   delete[] undistortedImage;

   if( gridImage != NULL )
      delete[]gridImage;
}


void 
Visualiser::ShowPoints( VWSLAM::PoseQuality quality, std::string title, const VWSLAM::VWSLAMStatus & status )
{   
#ifdef OPENCV_ENABLED
   cv::Mat rview;
   DrawLabelledImage( quality, undistortedImage, imageWidth, imageHeight, status, rview );

   //char imageName[256];
   //sprintf( imageName, "%s_labelledImage_%" PRId64 ".png", title.c_str(), poseWithTime.timestamp );
   //cv::imwrite( imageName, rview );
#endif

#ifndef ARM_BASED
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
#endif

#ifndef ARM_BASED
   /*cv::Mat doubleview;
   cv::resize( rview, doubleview, cv::Size( 2 * gridHeight, 2 * gridWidth ) );*/

   cv::imshow( "grid map", rview );
   cv::waitKey( 1 );
#endif

   // we always release the memory here. Please allocate another memory to hold if necessary
   if( gridImage != NULL )
      delete[]gridImage;
   gridImage = NULL;

   return;
}

#ifdef OPENCV_ENABLED
void Visualiser::DrawLabelledImage( VWSLAM::PoseQuality quality, const uint8_t * image, int widthFrame, int heightFrame, const VWSLAM::VWSLAMStatus & status, cv::Mat & rview )
{
   rview = cv::Mat( heightFrame, widthFrame, CV_8UC1 );
   memcpy( rview.data, image, heightFrame* widthFrame );

   cv::cvtColor( rview, rview,cv::COLOR_GRAY2BGR );
   
   int obsNum = status._MatchedMapPointNum + status._MisMatchedMapPointNum;
   if( obsNum > 0 )
   {
      if( quality != VWSLAM::QUALITY_FAILED )
      {
         cv::Point2f imagePoint;
         for( int i = 0; i < obsNum; i++ )
         {
            imagePoint.x = status.observationBuf[i].x;
            imagePoint.y = status.observationBuf[i].y;
            if( status.observationBuf[i].s == VWSLAM::TrackedObservation::MATCHING_OK )
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
   
   char strFrame[50];
   cv::Scalar color( 255, 0, 0 );
   if( VWSLAM::QUALITY_FAILED == quality ||
       VWSLAM::QUALITY_INITIALIZING == quality
       || VWSLAM::QUALITY_SCALEESTIMATION == quality
       )
   {
      color = cv::Scalar( 0, 0, 255 ); //red
   }
   else if( quality == VWSLAM::QUALITY_BAD )
   {
      color = cv::Scalar( 0, 255, 255 );
   }

   static int frameIndex = 0;
   frameIndex++;

   snprintf( strFrame, 30, "BR:%3d, KF:%5d", (int32_t)status._Brightness, status._KeyframeNum );
   //putText( rview, std::string( strFrame ), cv::Point2f( widthFrame - 290.0f, heightFrame - 30.0f ), cv::FONT_HERSHEY_COMPLEX, 0.6, color );
   putText( rview, std::string( strFrame ), cv::Point2f( widthFrame - 290.0f, heightFrame - 30.0f ), cv::FONT_HERSHEY_COMPLEX, 0.6, color );
   snprintf( strFrame, 50, "FrameIndex = %4d, Mismatched: %3d, Matched: %3d", frameIndex, status._MisMatchedMapPointNum, status._MatchedMapPointNum );
   putText( rview, std::string( strFrame ), cv::Point2f( widthFrame - 600.0f, 30.0f ), cv::FONT_HERSHEY_COMPLEX, 0.6, color );

   return;
}
#endif
