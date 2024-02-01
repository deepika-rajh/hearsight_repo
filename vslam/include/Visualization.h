/*****************************************************************************
@copyright
Copyright (c) 2020-2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/
#ifndef _VISUALIZATION_H_
#define _VISUALIZATION_H_

#include <thread>
#include <mutex>
#include "rvVWSLAM.h"
#include "rvVIO.h"
#ifdef OPENCV_ENABLED
#include "opencv2/opencv.hpp"
#endif

typedef struct
{
    float _Brightness = 0.f;
    int32_t _KeyframeNum = 0;
    int32_t _MatchedMapPointNum = 0;
    int32_t _MisMatchedMapPointNum = 0;
    RV_TrackedObservation* _ObservationBuf = NULL;
} rvVWSLAMStatus;

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

   int* getVisWidthAddr()
   {
	   return &gridWidth;
   }

   int& getVisHeight()
   {
      return gridHeight;
   }

   int* getVisHeightAddr()
   {
	   return &gridHeight;
   }

   unsigned char ** getVisData()
   {
      return &gridImage;
   }

   void ShowPoints( RV_VSLAM_TRACKING_STATE quality, std::string title, const rvVWSLAMStatus & status );

   void ShowVIOPoints(const uint8_t* image,RV_VSLAM_TRACKING_STATE quality, std::string title, const int& pointNum, const rvVISLAMMapPoint* pPoints);

   void ShowGridMap(int64_t timestamp, int poseX, int poseY);

   void WriteGrayBitmap( unsigned char *iImgData, char *iImgName, int iWidth, int iHeight, int iPosX, int iPosY, int iFullLine, int Flag );

private:
   int imageHeight;
   int imageWidth;
   uint8_t * undistortedImage;

   int gridHeight;
   int gridWidth;
   uint8_t * gridImage;

   std::mutex occupancyGridMutex;
   int occupancyGridHeight;
   int occupancyGridWidth;
   uint8_t * occupancyGridImage;

#ifdef OPENCV_ENABLED
   void DrawVIOLabelledImage(RV_VSLAM_TRACKING_STATE quality, const uint8_t* image, int widthFrame, int heightFrame, const int& pointNum, const rvVISLAMMapPoint* pPoints, cv::Mat& rview);

   void DrawLabelledImage(RV_VSLAM_TRACKING_STATE quality, const uint8_t * image, int widthFrame, int heightFrame, const rvVWSLAMStatus & status, cv::Mat & rview );
#endif
};

#endif //VISUALIZATION
