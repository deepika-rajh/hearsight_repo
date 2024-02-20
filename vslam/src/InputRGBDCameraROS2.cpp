/*****************************************************************************
@copyright
Copyright (c) 2022-2023 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include "InputRGBDCameraROS2.h"
#include <cv_bridge/cv_bridge.h>

//bool readStereoCameraParameter(const char* cameraID, rvStereoCamera& configuration, rvStereoRectCamera & outputCamera);

const uint32_t kQueueSize = 1;
const uint32_t kSyncQueueSize = 3;
const char* kLeftImageTopic = "left_image";
const char* kRightImageTopic = "right_image";

InputRGBDCameraROS2::InputRGBDCameraROS2(rclcpp::Node::SharedPtr const &node_, const std::string & config ): node(node_)
{
    cameraParams.cameraType = rvGrayDepth;
    if (ReadCameraConfig(config, cameraParams))
    {
        printf("load camera parameter successfully\n");
        gotCameraPara = true;
        rgbInfoSub = NULL;
        rgb_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image> >(node, "/camera/color/image_raw");
        depth_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image> >(node, "/camera/aligned_depth_to_color/image_raw");
        syncApproximate = std::make_shared<RGBDSync>(RGBDSync(10), *rgb_sub, *depth_sub);
        syncApproximate->registerCallback(bind(&InputRGBDCameraROS2::callback, this, std::placeholders::_1, std::placeholders::_2));
    }
    else
    {
        rgbInfoSub = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
                     std::string("/camera/color/camera_info"), 10, bind(&InputRGBDCameraROS2::rgbInfo_callback, this, std::placeholders::_1));
        gotCameraPara = false;
        rgb_sub = NULL;
        depth_sub = NULL;
    }

}

InputRGBDCameraROS2::~InputRGBDCameraROS2()
{

}

void InputRGBDCameraROS2::callback(const sensor_msgs::msg::Image::ConstSharedPtr& image,
	const sensor_msgs::msg::Image::ConstSharedPtr& depth)
{
    if ( image != nullptr )
    {
        //rclcpp::Time t = left_image->header.stamp;
        //std::cout << "image time: " << image->header.stamp.sec + image->header.stamp.nanosec*1e-9 << std::endl;
        printf("image time %10.6f\n",image->header.stamp.sec + image->header.stamp.nanosec*1e-9);
    }
    else
    {
        std::cout << "recieve empty" << std::endl;
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRGB;
    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrRGB = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::RGB8);
        //cv_ptrRGB = cv_bridge::toCvCopy(image);
    }
    catch (cv_bridge::Exception& e)
    {
        printf("rgb error\n");
        return;
    }

    try
    {
        cv_ptrD = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
        printf("depth error\n");
        return;
    }

    printf("call callback\n");
    rclcpp::Time t = image->header.stamp;
    cvtColor(cv_ptrRGB->image, grayImage, cv::COLOR_RGB2GRAY);
    callback_(t.nanoseconds(), grayImage.data, (const uint16_t *)cv_ptrD->image.data);
}

void InputRGBDCameraROS2::rgbInfo_callback(const sensor_msgs::msg::CameraInfo::SharedPtr rgbInfo)
{
   if (gotCameraPara)
       return;

   printf("process camera info\n");

   cameraParams.imageFormat = Y_ONLY_FORMAT;
   cameraParams.cameraType = rvGrayDepth;
   cameraParams.stereo.camera[0].pixelWidth = rgbInfo->width;
   cameraParams.stereo.camera[0].pixelHeight = rgbInfo->height;
   cameraParams.stereo.camera[0].pixelStride = rgbInfo->width;
   cameraParams.stereo.camera[0].principalPoint[0] = rgbInfo->k[2];
   cameraParams.stereo.camera[0].principalPoint[1] = rgbInfo->k[5];
   cameraParams.stereo.camera[0].focalLength[0] = rgbInfo->k[0];
   cameraParams.stereo.camera[0].focalLength[1] = rgbInfo->k[4];
   cameraParams.stereo.camera[0].distortionModel = rvDistortionModel::Polynomial5;
   size_t i = 0;
   for (; i<5; i++)
   {
       cameraParams.stereo.camera[0].distortion[i] = rgbInfo->d[i];
   }
   for (; i<8; i++)
   {
       cameraParams.stereo.camera[0].distortion[i] = 0;
   }

   cameraParams.stereoRect.camera[0].initialized = false;

   cameraParams.stereoRect.camera[0].pixelHeight = cameraParams.stereo.camera[0].pixelHeight;
   cameraParams.stereoRect.camera[0].pixelWidth = cameraParams.stereo.camera[0].pixelWidth;

   gotCameraPara = true;


   rgb_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image> >(node, "/camera/color/image_raw");
   depth_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image> >(node, "/camera/aligned_depth_to_color/image_raw");

   syncApproximate = std::make_shared<RGBDSync>(RGBDSync(10), *rgb_sub, *depth_sub);
   syncApproximate->registerCallback(bind(&InputRGBDCameraROS2::callback, this, std::placeholders::_1, std::placeholders::_2));

   rgbInfoSub = NULL;

}

bool InputRGBDCameraROS2::ReadCameraConfig( const std::string & filename, rvCameraParams & cameraParams )
{
   printf("***ZYM*** %s\n",filename.c_str());
   cv::FileStorage fs_read( filename.c_str(), cv::FileStorage::READ );

   if (!fs_read.isOpened())
      return false;

   cameraParams.cameraType = rvGrayDepth;
   cameraParams.imageFormat = Y_ONLY_FORMAT;

   int height, width;
   fs_read["image_width"] >> width;
   fs_read["image_height"] >> height;
   cv::Size imageSize(width, height);
   printf("***ZYM*** width %d height %d\n", imageSize.width, imageSize.height);
   cv::Mat cameraIntrinsics0, distortion0;
   std::string distortionModel;
   fs_read["distortion_model"]>>distortionModel;
   fs_read["camera_matrix"] >> cameraIntrinsics0;
   fs_read["distortion_coefficients"] >> distortion0;
   getCameraSetting(distortionModel, cameraIntrinsics0, distortion0, imageSize, cameraParams.stereo.camera[0]);
   cv::Mat p, r;
   if (!fs_read["projection_matrix"].empty())
   {
      fs_read["projection_matrix"] >> p;
      fs_read["rectification_matrix"] >> r;

      cameraParams.stereoRect.camera[0].initialized = true;
      cameraParams.stereoRect.camera[0].pixelHeight = imageSize.height;
      cameraParams.stereoRect.camera[0].pixelWidth = imageSize.width;
      for (size_t i=0; i<3; i++)
      {
          for (size_t j=0; j<3; j++)
          {
              cameraParams.stereoRect.camera[0].P[i][j] = p.at<double>(i,j);
              cameraParams.stereoRect.camera[0].R[i][j] = r.at<double>(i,j);
          }
          cameraParams.stereoRect.camera[0].P[i][3] = p.at<double>(i,3);
      }
   }
   else
   {
       cameraParams.stereoRect.camera[0].initialized = false;
       cameraParams.stereoRect.camera[0].pixelHeight = imageSize.height;
       cameraParams.stereoRect.camera[0].pixelWidth = imageSize.width;
   }

   cameraParams.stereoRect.camera[1].initialized = false;
   cameraParams.stereoRect.camera[1].pixelHeight = imageSize.height;
   cameraParams.stereoRect.camera[1].pixelWidth = imageSize.width;

   return true;
}


void InputRGBDCameraROS2::getCameraSetting(const std::string & distortionModel, const cv::Mat & intrinsics, const cv::Mat & distortion, const cv::Size & imageSize, rvCameraIntrinsic & cameraConfig)
{
   cameraConfig.focalLength[0] = (float32_t)intrinsics.at<double>( 0, 0 );
   cameraConfig.focalLength[1] = (float32_t)intrinsics.at<double>( 1, 1 );
   cameraConfig.principalPoint[0] = (float32_t)intrinsics.at<double>( 0, 2 );
   cameraConfig.principalPoint[1] = (float32_t)intrinsics.at<double>( 1, 2 );

   cameraConfig.pixelWidth = imageSize.width;
   cameraConfig.pixelHeight = imageSize.height;

   memset( cameraConfig.distortion, 0, sizeof( cameraConfig.distortion ) );
   if (distortionModel == "fisheye")
   {
       cameraConfig.distortionModel = rvDistortionModel::FisheyeModel4;
   }
   else
   {
	  printf("step 0.1\n");
      switch(distortion.cols)
      {
          case 5:
              cameraConfig.distortionModel = rvDistortionModel::Polynomial5;
              break;
          case 4:
              cameraConfig.distortionModel = rvDistortionModel::Polynomial4;
              break;
          default:
          case 8:
              cameraConfig.distortionModel = rvDistortionModel::RationalModel8;
              break;
      }
   }

   for (int i=0; i<distortion.cols; i++ )
      cameraConfig.distortion[i] = (float32_t)distortion.at<double>( i );
}
