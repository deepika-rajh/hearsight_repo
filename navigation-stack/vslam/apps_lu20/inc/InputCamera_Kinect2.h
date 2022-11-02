/*****************************************************************************
@copyright
Copyright (c) 2020-2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef _INPUT_CAMERA_KINECT2_H_
#define _INPUT_CAMERA_KINECT2_H_ 

#include "rvCamera.h"
#include <functional>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <mutex>
#include <opencv2/opencv.hpp>

#include <VSLAMSystem.h>

class InputCamera_Kinect2: public VSLAMCameraInterface
{
public:
    using CallbackFunc = std::function<void(const int64_t timestamp, const uint8_t * imageBuf, const uint16_t * depthBuf)>;
    using ImageSubscriber = message_filters::Subscriber<sensor_msgs::Image>;
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>;
    using TimeSynchronizer = message_filters::Synchronizer<SyncPolicy>;

    InputCamera_Kinect2(ros::NodeHandle* node_handle, const std::string & config);
    ~InputCamera_Kinect2();

    void addCallback(CameraCallback callback)
    {
        callback_ = callback;
    }

	int start()
	{
		return 0;
	}

	void stop()
	{
	}

   const rvCameraParams & getCameraConfiguration( ) const;

private:
    void callback(const sensor_msgs::ImageConstPtr& left_image,
        const sensor_msgs::ImageConstPtr& depth_image);
		
	bool readCameraParameter( const std::string & cameraID, rvCameraParams & configuration );

    std::unique_ptr<ImageSubscriber> left_image_sub_;
    std::unique_ptr<ImageSubscriber> depth_image_sub_;
    std::unique_ptr<TimeSynchronizer> time_synchronizer_;
    CameraCallback callback_;
    rvCameraParams cameraParams;
};

#endif //_INPUT_STEREO_CAMERA_ROS_H_
