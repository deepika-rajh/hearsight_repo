/*****************************************************************************
@copyright
Copyright (c) 2022-2023 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/
#ifndef _INPUT_MONO_CAMERA_ROS2_H_
#define _INPUT_MONO_CAMERA_ROS2_H_

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

#include <opencv2/opencv.hpp>

#include "VSLAMSystem.h"


class InputMonoCameraROS2: public CameraInterface
{
public:
	using CallbackFunc = std::function<void(const int64_t timestamp, const uint8_t * imageBuf, const uint16_t * depthBuf)>;

	InputMonoCameraROS2(rclcpp::Node::SharedPtr const & node_);
	~InputMonoCameraROS2();

    void addCallback(CameraCallback callback)
    {
        printf("Camera add callback\n");
        callback_ = callback;
    }

    bool start()
    {
        printf("camera start\n");

        size_t i = 0;
        while (!gotCameraPara)
        {
            rclcpp::spin_some( node );
            printf("no camera info %lu\n", i);
            usleep(1000);
            i ++;
        }
        printf("got camera para\n");
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
    void callback(const sensor_msgs::msg::Image::ConstSharedPtr& image);

    void rgbInfo_callback(const sensor_msgs::msg::CameraInfo::SharedPtr rgbInfo);

    image_transport::Subscriber rgb_sub;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr rgbInfoSub;

    CameraCallback callback_;
    rvCameraParams cameraParams;
    bool gotCameraPara;

    rclcpp::Node::SharedPtr node;
    cv::Mat grayImage;
};

#endif //_INPUT_MONO_CAMERA_ROS2_H_
