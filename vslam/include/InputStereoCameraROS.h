/*****************************************************************************
@copyright
Copyright (c) 2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/
#ifndef _INPUT_STEREO_CAMERA_ROS_H_
#define _INPUT_STEREO_CAMERA_ROS_H_

#include <functional>
#include <vector>
#include <thread> 
#include <memory>

#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "CameraInterface.h"
#include "rvCamera.h"

#include <opencv2/opencv.hpp>

#include "VSLAMSystem.h"

class InputStereoCameraROS: public CameraInterface
{
public:
	using ImageSubscriber = message_filters::Subscriber<sensor_msgs::Image>;
	using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>;
	using TimeSynchronizer = message_filters::Synchronizer<SyncPolicy>;

	InputStereoCameraROS(ros::NodeHandle* node_handle, const std::string & config);
	~InputStereoCameraROS();

    void addCallback(CameraCallback callback)
    {
        callback_ = callback;
    }

	bool start()
	{
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
    void callback(const sensor_msgs::ImageConstPtr& left_image,
        const sensor_msgs::ImageConstPtr& right_image);

    void ReadCameraConfig( const std::string & filename, rvCameraParams & cameraParams );
    void getCameraSetting(const cv::Mat & intrinsics, const cv::Mat & distortion, const cv::Size & imageSize, const std::string & distortionString, rvCameraIntrinsic & cameraConfig);

    std::unique_ptr<ImageSubscriber> left_image_sub_;
    std::unique_ptr<ImageSubscriber> right_image_sub_;
    std::unique_ptr<TimeSynchronizer> time_synchronizer_;
    CameraCallback callback_;
    rvCameraParams cameraParams;
};

#endif //_INPUT_STEREO_CAMERA_ROS_H_
