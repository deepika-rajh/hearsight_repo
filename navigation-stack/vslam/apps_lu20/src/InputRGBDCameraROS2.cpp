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

InputRGBDCameraROS2::InputRGBDCameraROS2(rclcpp::Node::SharedPtr const &node_ ): node(node_)
{
    rgbInfoSub = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
    std::string("/camera/color/camera_info"), 10, bind(&InputRGBDCameraROS2::rgbInfo_callback, this, std::placeholders::_1));
    cameraParams.cameraType = rvStereo;

    gotCameraPara = false;

    rgb_sub = NULL;
    depth_sub = NULL;
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

   for(size_t i = 0; i< 5; i++)
   {
       cameraParams.stereo.camera[0].distortion[i] = rgbInfo->d[i];
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
