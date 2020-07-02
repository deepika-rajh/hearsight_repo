/*****************************************************************************
@copyright
Copyright (c) 2020 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/
#ifndef _VISUALIZATION_H_
#define _VISUALIZATION_H_

#include <thread>
#include "VWSLAM.h"
#include "opencv2/opencv.hpp"

class Visualiser
{
public:
   Visualiser( int imageWidth, int imageHeight );
   ~Visualiser();

   uint8_t * getUndistortedImageBuf()
   {
      return undistortedImage;
   }

   int getImageHeight()
   {
      return imageHeight;
   }

   int getImageWidth()
   {
      return imageWidth;
   }

   int& getVisWidth()
   {
      return gridWidth;
   }

   int& getVisHeight()
   {
      return gridHeight;
   }

   unsigned char ** getVisData()
   {
      return &gridImage;
   }

   void ShowPoints( VWSLAM::PoseQuality quality, std::string title, const VWSLAM::VWSLAMStatus & status );

   void ShowGridMap();

private:
   int imageHeight;
   int imageWidth;
   uint8_t * undistortedImage;

   int gridHeight;
   int gridWidth;
   uint8_t * gridImage;

   void DrawLabelledImage( VWSLAM::PoseQuality quality, const uint8_t * image, int widthFrame, int heightFrame, const VWSLAM::VWSLAMStatus & status, cv::Mat & rview );
};

#endif //VISUALIZATION
