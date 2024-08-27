/*******************************************************************************
@copyright
Copyright (c) 2022-2023 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/
#ifndef __DFS_ROS2__
#define __DFS_ROS2__

#include "rvDFS.h"
#include "rv_dfs_base.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include <chrono>
#include <condition_variable>
#include <image_transport/image_transport.hpp>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <thread>

class DFSRosTwoNode : public rclcpp::Node {
public:
  DFSRosTwoNode(const rmw_qos_profile_t &qos);
  void init_dfs();
  ~DFSRosTwoNode();

private:
  void processTwoImgs();
  void processStitchImg();
  void processColor();
  void callbackTwoImgs(const sensor_msgs::msg::Image::ConstSharedPtr &img_p1,
                       const sensor_msgs::msg::Image::ConstSharedPtr &img_p2);
  void callbackStitchImg(const sensor_msgs::msg::Image::SharedPtr img_p1);
  void callbackColor(const sensor_msgs::msg::Image::SharedPtr color_p);
  void parseCameraInfo(const sensor_msgs::msg::CameraInfo &cm1,
                       const sensor_msgs::msg::CameraInfo &cm2);
  void toShort16(cv::Mat &fmat, cv::Mat &smat);
  void toColorByUchar(cv::Mat &fmat, cv::Mat &umat);
  void initCloudMsg();
  void updateCloudMsg();
  std_msgs::msg::Header genHeader(const std::string &frame_id);

  void debug_info(const std::string &s);
  void debug_img();

  std::condition_variable con_stereo;
  std::condition_variable con_color;
  std::mutex mtx_stereo;
  std::mutex mtx_color;
  std::shared_ptr<std::thread> t_stereo;
  std::shared_ptr<std::thread> t_color;

  rmw_qos_profile_t qos_prf;
  rclcpp::Node::SharedPtr np;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr stitch_sub;
  message_filters::Subscriber<sensor_msgs::msg::Image> img1_sub;
  message_filters::Subscriber<sensor_msgs::msg::Image> img2_sub;

  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image, sensor_msgs::msg::Image>
      MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  std::shared_ptr<Sync> sy;

  cv_bridge::CvImagePtr cv_p1;
  cv_bridge::CvImagePtr cv_p2;
  cv_bridge::CvImagePtr cv_p3;
  cv::Mat now_img1;
  cv::Mat now_img2;
  std_msgs::msg::Header img_head;
  sensor_msgs::msg::PointCloud2 cloud_msg;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_pub;
  image_transport::Publisher disp_pub;
  image_transport::Publisher dep_pub;
  image_transport::Publisher left_pub;
  image_transport::Publisher color_pub;
  image_transport::Publisher rectl_pub;
  image_transport::Publisher rectr_pub;

  rvDFSMode dfs_mode;
  rvDFSParameter dfs_param;
  rvDFSParamRuntime dfs_run_param;
  rvStereoCamera stereo_param;

  rvDFSInputParam in;
  rvDFSOutputParam out;

  std::shared_ptr<rv_dfs::DFSBase<float>> dfs_base;
  bool init_flag;
  bool color_flag;
  bool calib_file_flag;
  bool stitch_img_flag;
  bool color_point_flag;
  std::string calib_file_path;

  bool get_stereo;
  bool get_color;

  int md;
  int disp_min;
  int disp_lvl;

  int width;
  int height;
  int stride;
  int pixel;
  float fx, fy;
  float cx, cy;

  std::string disp_id;
  std::string dep_id;
  std::string left_id;
  std::string point_id;
  std::string rectl_id;
  std::string rectr_id;

  uint8_t *imgl;
  uint8_t *imgr;
  cv::Mat dispImg;
  cv::Mat depImg;
  cv::Mat dispColor;
  cv::Mat depShort;
  cv::Mat rectl;
  cv::Mat rectr;
  rv_dfs::PointCloudType<float> pc;
  std::chrono::high_resolution_clock::time_point start;
  std::chrono::high_resolution_clock::time_point stop;
  std::chrono::duration<double> elapsed;
};

#endif