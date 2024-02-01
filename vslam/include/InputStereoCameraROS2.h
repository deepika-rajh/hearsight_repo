/*****************************************************************************
@copyright
Copyright (c) 2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/
#ifndef _INPUT_STEREO_CAMERA_ROS2_H_
#define _INPUT_STEREO_CAMERA_ROS2_H_

#include <functional>
#include <vector>
#include <thread> 
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include <sensor_msgs/msg/camera_info.hpp>

#include "CameraInterface.h"
#include "rvCamera.h"

#include <opencv2/opencv.hpp>

#include "VSLAMSystem.h"

typedef message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>  StereoSync;

class InputStereoCameraROS2: public CameraInterface
{
public:
	using CallbackFunc = std::function<void(const int64_t timestamp, const uint8_t * imageBuf, const uint8_t * rightBuf)>;

    InputStereoCameraROS2(rclcpp::Node::SharedPtr const & node_, const std::string & config = "");
    ~InputStereoCameraROS2();

    void addCallback(CameraCallback callback)
    {
		printf("Camera add callback\n");
        callback_ = callback;
    }

	bool start()
	{
		printf("camera start\n");
		
		size_t i = 0;
        while (!gotRightCameraPara || !gotLeftCameraPara)
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
    void callback(const sensor_msgs::msg::Image::ConstSharedPtr& image,
	const sensor_msgs::msg::Image::ConstSharedPtr& depth );
	
    void leftInfo_callback(const sensor_msgs::msg::CameraInfo::SharedPtr rgbInfo);
    void rightInfo_callback(const sensor_msgs::msg::CameraInfo::SharedPtr rgbInfo);
    
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > left_sub;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > right_sub;
	
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr leftInfoSub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr rightInfoSub;
    
    std::shared_ptr<StereoSync> syncApproximate;
    
    CameraCallback callback_;
    rvCameraParams cameraParams;
    bool gotLeftCameraPara = false;
    bool gotRightCameraPara = false;
    
    void copyInfo(const sensor_msgs::msg::CameraInfo::SharedPtr rgbInfo, rvRectCameraConfiguration &camera);
    void copyInfo(const sensor_msgs::msg::CameraInfo::SharedPtr rgbInfo, rvCameraIntrinsic &camera);
    void makeSncPolocy();
    rclcpp::Node::SharedPtr node;
    cv::Mat grayImage;

	bool ReadCameraConfig( const std::string & filename, rvCameraParams & cameraParams );
	void getCameraSetting( const std::string & distortionModel, const cv::Mat & intrinsics, const cv::Mat & distortion, const cv::Size & imageSize, rvCameraIntrinsic & cameraConfig);
};

#endif //_INPUT_STEREO_CAMERA_ROS_H_
