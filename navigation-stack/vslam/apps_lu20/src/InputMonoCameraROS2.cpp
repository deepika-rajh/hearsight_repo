/*****************************************************************************
@copyright
Copyright (c) 2022-2023 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include "InputMonoCameraROS2.h"
#include <cv_bridge/cv_bridge.h>

const uint32_t kQueueSize = 1;
const uint32_t kSyncQueueSize = 3;
const char* kLeftImageTopic = "left_image";
const char* kRightImageTopic = "right_image";

InputMonoCameraROS2::InputMonoCameraROS2(rclcpp::Node::SharedPtr const &node_ ): node(node_)
{
    rgbInfoSub = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
    std::string("/camera/color/camera_info"), 10, bind(&InputMonoCameraROS2::rgbInfo_callback, this, std::placeholders::_1));
    cameraParams.cameraType = rvMonocular;
    gotCameraPara = false;
}

InputMonoCameraROS2::~InputMonoCameraROS2()
{

}

void InputMonoCameraROS2::callback(const sensor_msgs::msg::Image::ConstSharedPtr& image)
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
    try
    {
        cv_ptrRGB = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception& e)
    {
        printf("rgb error\n");
        return;
    }

    printf("call callback\n");
    rclcpp::Time t = image->header.stamp;
    cvtColor(cv_ptrRGB->image, grayImage, cv::COLOR_RGB2GRAY);
    callback_(t.nanoseconds(), grayImage.data, (const uint16_t *)NULL);
}

void InputMonoCameraROS2::rgbInfo_callback(const sensor_msgs::msg::CameraInfo::SharedPtr rgbInfo)
{
   if (gotCameraPara)
      return;

   printf("process camera info\n");

   cameraParams.imageFormat = Y_ONLY_FORMAT;
   cameraParams.cameraType = rvMonocular;
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

   printf("make subsriber of color image\n");
   rgb_sub = image_transport::create_subscription(node.get(), "/camera/color/image_raw", bind(&InputMonoCameraROS2::callback, this, std::placeholders::_1), "raw", rmw_qos_profile_default);
   rgbInfoSub = NULL;

}
