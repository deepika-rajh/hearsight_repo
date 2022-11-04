/*******************************************************************************
@copyright
Copyright (c) 2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/
#ifndef __DFS_ROS2__
#define __DFS_ROS2__


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "image_transport/image_transport.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "rvDFS.h"
#include "rv_dfs_base.h"
#include <gst/gst.h>

class DFSNode : public rclcpp::Node
{
public:
    DFSNode();
    ~DFSNode();

private:
    static GstFlowReturn new_sample_cb (GstElement *sink, gpointer userdata);
    inline void setupImageMsgHeader(sensor_msgs::msg::Image::SharedPtr imgMsg);
    inline void publishFrames();
    inline void setupCameraInfoMsgHeader();

    rclcpp::Node::SharedPtr g_node;
    image_transport::Publisher    left_image_pub;
    image_transport::Publisher    right_image_pub;
    image_transport::Publisher    disparity_pub;
    image_transport::Publisher    depth_pub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_color_pub;
    sensor_msgs::msg::Image::SharedPtr monoLeftImageMsg;
    sensor_msgs::msg::Image::SharedPtr monoRightImageMsg;
    sensor_msgs::msg::Image::SharedPtr disparityImageMsg;
    sensor_msgs::msg::Image::SharedPtr depthImageMsg;
    sensor_msgs::msg::PointCloud2 pointCloudMsg;
    sensor_msgs::msg::PointCloud2 pointCloudColorMsg;
    sensor_msgs::msg::CameraInfo::SharedPtr cameraInfoMsg;

    std::shared_ptr<rv_dfs::DFSBase> dfs_base;
    rvDFSMode dfs_mode;
    int width;
    int height;
    int stride;
    int pixel;
    rvDFSParameter dfs_parameter;
    rvStereoCamera stereo_parameter;
    rvStereoCamera rectified_stereo_parameter;
    float* disp;
    float* depth;
    uint8_t* imgL;
    uint8_t* imgR;
    PointCloudType pcl;
    PointCloudColorType pcc;
    cv::Mat monoLeftImage;
    cv::Mat monoRightImage;
    cv::Mat disparityImage;
    cv::Mat depthImage;
    sensor_msgs::msg::CameraInfo cameraInfo;
    
}
