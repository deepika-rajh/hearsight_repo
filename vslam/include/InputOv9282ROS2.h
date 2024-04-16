/*****************************************************************************
Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
All rights reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/
#ifndef _INPUT_OV9282_ROS2_H_
#define _INPUT_OV9282_ROS2_H_

#include <functional>
#include <vector>
#include <thread>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <sensor_msgs/image_encodings.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include "CameraInterface.h"
#include "rvCamera.h"
#include "qrb_ros_camera/qrb_ros_image_type.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class InputOv9282ROS2: public CameraInterface
{
public:
    using CallbackFunc = std::function<void(const int64_t timestamp, const uint8_t * imageBuf, const uint16_t * depthBuf)>;

    InputOv9282ROS2(rclcpp::Node & node_, const std::string & config );
    ~InputOv9282ROS2();

    void addCallback(CameraCallback callback)
    {
        printf("ov9282 add callback\n");
        callback_ = callback;
    }

    bool start()
    {
        printf("ov9282 start\n");

        size_t i = 0;
        while (!gotCameraPara)
        {
            printf("no ov9282 info %lu\n", i);
            usleep(1000);
            i ++;
        }
        printf("got ov9282 para\n");
        return true;
    }

    bool stop()
    {
        return true;
    }

    const rvCameraParams & getCameraConfiguration( ) const
    {
        return cameraParams;
    }

private:
    void callback(const qrb_ros_type::QrbRosImageTypeAdapter& image);

    void rgbInfo_callback(const sensor_msgs::msg::CameraInfo::SharedPtr rgbInfo);

	bool ReadCameraConfig( const std::string & filename, rvCameraParams & cameraParams );

	void getCameraSetting(const std::string & distortionModel, const cv::Mat & intrinsics, const cv::Mat & distortion, const cv::Size & imageSize, rvCameraIntrinsic & cameraConfig);

    rclcpp::Subscription<qrb_ros_type::QrbRosImageTypeAdapter>::SharedPtr rgb_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr rgbInfoSub;

    CameraCallback callback_;
    rvCameraParams cameraParams;
    bool gotCameraPara;

    rclcpp::Node & node;
    cv::Mat grayImage;
};

#endif //_INPUT_STEREO_CAMERA_ROS_H_
