/*****************************************************************************
@copyright
Copyright (c) 2023-2024 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include "dfs_ros2.h"

#include "dfs_factory.h"
#include <fstream>
#include <functional> // std::bind
#include <rclcpp/serialization.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <string>

namespace enc = sensor_msgs::image_encodings;
using std::placeholders::_1;
using std::placeholders::_2;

static bool cm_info_flag = false;
static bool extrin_flag = false;
static int dfs_cnt = 0;
static std::string dbg_dir = "/data/dbg_dir/dfs_ros/";

static const rmw_qos_profile_t my_qos = {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    3,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false};

static void getMinMaxThenColor(cv::Mat &raw, cv::Mat &out) {
  double min;
  double max;
  cv::minMaxIdx(raw, &min, &max);
  double scale = 255. / (max - min);
  raw.convertTo(raw, CV_8UC1, scale, -min * scale);
  cv::applyColorMap(raw, out, cv::COLORMAP_JET);
}

static void
writePLYPointCloud(const std::string &ply_file_path,
                   const rv_dfs::PointCloudType<float> &pointCloud) {
  std::ofstream o_st(ply_file_path);
  o_st << "ply" << std::endl;
  o_st << "format ascii 1.0" << std::endl;
  o_st << "element vertex " << pointCloud.size() << std::endl;
  o_st << "property float x" << std::endl;
  o_st << "property float y" << std::endl;
  o_st << "property float z" << std::endl;
  o_st << "end_header" << std::endl;
  for (const auto &pc : pointCloud) {
    o_st << std::fixed << std::setprecision(2) << pc[0] << " " << pc[1] << " "
         << pc[2] << std::endl;
  }
}

static rvStereoCamera
importStereoCalData(const std::string &file) // Translation should be millimeter
{
  rvStereoCamera param;
  param.camera[0].pixelWidth = 0;
  param.camera[0].pixelHeight = 0;
  // load parameter files
  cv::FileStorage fs(file, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    return param;
  }

  cv::Mat camera_mat_left;
  fs["Camera_Matrix1"] >> camera_mat_left;
  param.camera[0].focalLength[0] = (float32_t)camera_mat_left.at<double>(0, 0);
  param.camera[0].focalLength[1] = (float32_t)camera_mat_left.at<double>(1, 1);
  param.camera[0].principalPoint[0] =
      (float32_t)camera_mat_left.at<double>(0, 2);
  param.camera[0].principalPoint[1] =
      (float32_t)camera_mat_left.at<double>(1, 2);
  cv::Mat distortion_left;
  fs["Distortion_Coefficients1"] >> distortion_left;

  cv::Mat camera_mat_right;
  fs["Camera_Matrix2"] >> camera_mat_right;
  param.camera[1].focalLength[0] = (float32_t)camera_mat_right.at<double>(0, 0);
  param.camera[1].focalLength[1] = (float32_t)camera_mat_right.at<double>(1, 1);
  param.camera[1].principalPoint[0] =
      (float32_t)camera_mat_right.at<double>(0, 2);
  param.camera[1].principalPoint[1] =
      (float32_t)camera_mat_right.at<double>(1, 2);

  cv::Mat distortion_right;
  fs["Distortion_Coefficients2"] >> distortion_right;

  cv::FileNode image_size_node = fs["Image_Size"];
  std::vector<int> image_size;
  image_size_node >> image_size;
  param.camera[0].pixelWidth = image_size[0];
  param.camera[0].pixelHeight = image_size[1];
  param.camera[1].pixelWidth = image_size[0];
  param.camera[1].pixelHeight = image_size[1];

  std::string distortionModel;
  fs["distortion_model"] >> distortionModel;
  int ModelType;
  fs["M1"] >> ModelType;

  cv::Mat R;
  fs["R"] >> R;
  // Translation should be millimeter
  cv::Mat T;
  fs["T"] >> T;
  cv::Vec3d rot_rodrigues;
  if (R.total() > 3)
    cv::Rodrigues(R, rot_rodrigues);
  for (int i = 0; i < 3; ++i) {
    param.translation[i] = (float32_t)T.at<double>(i, 0);
    if (R.total() > 3)
      param.rotation[i] = (float32_t)rot_rodrigues[i];
    else
      param.rotation[i] = (float32_t)R.at<double>(i, 0);
  }

  size_t len = distortion_left.rows * distortion_left.cols;
  for (int i = 0; i < 14; i++) {
    param.camera[0].distortion[i] = 0.0f;
    param.camera[1].distortion[i] = 0.0f;
  }

  rvDistortionModel distort_model = Polynomial5;
  if (len == 5)
    distort_model = Polynomial5;
  else if (len == 8)
    distort_model = RationalModel8;
  else if (len == 12)
    distort_model = ThinPrism12;
  else if (len == 14)
    distort_model = Tilted14;
  else if (len == 4) {
    if (strncmp(distortionModel.c_str(), "fisheye", 7) == 0 ||
        strcasecmp(distortionModel.c_str(), "equidistant") == 0)
      distort_model = FisheyeModel4;
    else
      distort_model = Polynomial4;
  }

  double *p1 = distortion_left.ptr<double>(0);
  double *p2 = distortion_right.ptr<double>(0);
  for (int i = 0; i < len; i++) {
    param.camera[0].distortion[i] = *(p1 + i);
    param.camera[1].distortion[i] = *(p2 + i);
  }
  param.camera[0].distortionModel = distort_model;
  param.camera[1].distortionModel = distort_model;

  return param;
}

DFSRosTwoNode::DFSRosTwoNode(const rmw_qos_profile_t &qos)
    : Node("dfs_ros2_node"), init_flag(false), color_flag(false),
      calib_file_flag(false), stitch_img_flag(false), color_point_flag(false),
      get_stereo(false), get_color(false) {
  np = std::shared_ptr<rclcpp::Node>(this);
  qos_prf = qos;

  disp_pub =
      image_transport::create_publisher(this, "/dfs_ros2/disparity_image", qos);
  dep_pub = image_transport::create_publisher(
      this, "/dfs_ros2/depth_raw",
      qos); // short 16bit, mm unit. same to RealSense.
  left_pub =
      image_transport::create_publisher(this, "/dfs_ros2/left_image", qos);
  point_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/dfs_ros2/point_cloud",
      rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos), qos));
  rectl_pub =
      image_transport::create_publisher(this, "/dfs_ros2/left_rect", qos);
  rectr_pub =
      image_transport::create_publisher(this, "/dfs_ros2/right_rect", qos);
}

void DFSRosTwoNode::init_dfs() {
  md = this->get_parameter("dfs_mode").as_int();
  disp_min = this->get_parameter("disparity_min").as_int();
  disp_lvl = this->get_parameter("disparity_level").as_int();
  color_flag = this->get_parameter("enable_color").as_bool();
  calib_file_flag = this->get_parameter("enable_calib_file").as_bool();
  stitch_img_flag = this->get_parameter("enable_stitch_img").as_bool();
  color_point_flag = this->get_parameter("enable_color_point").as_bool();
  calib_file_path = this->get_parameter("calib_file_path").as_string();
  printf("rv_dfs_ros2 md %d, disp_min %d, disp_lvl %d, color_flag %d \n", md,
         disp_min, disp_lvl, color_flag);
  printf("calib_file_flag %d, stitch_img_flag %d, color_point_flag %d\n",
         calib_file_flag, stitch_img_flag, color_point_flag);
  printf("calib_file_path %s\n", calib_file_path.c_str());

  if (md > 3 || md < 0)
    md = 0;
  if (disp_min < 0)
    disp_min = 0;
  if (disp_lvl > 256)
    disp_lvl = 256;

  if (calib_file_flag) // read from file
  {
    stereo_param = importStereoCalData(calib_file_path);
    if (stereo_param.camera[0].pixelWidth == 0) {
      std::cout << "Not right stereo calibration file error." << std::endl;
      return;
    }
    width = stereo_param.camera[0].pixelWidth;
    height = stereo_param.camera[0].pixelHeight;
    stride = width;
  } else {
    sensor_msgs::msg::CameraInfo cm1;
    sensor_msgs::msg::CameraInfo cm2;
    rclcpp::wait_for_message<sensor_msgs::msg::CameraInfo>(
        cm1, np, "/camera/infra1/camera_info"); // must rclcpp::Node::SharedPtr
    rclcpp::wait_for_message<sensor_msgs::msg::CameraInfo>(
        cm2, np, "/camera/infra2/camera_info");
    parseCameraInfo(cm1, cm2);
  }

  if (stitch_img_flag)
    stride = 2 * width;
  debug_info("first");
  pixel = width * height;

  dfs_param.mode = static_cast<rvDFSMode>(md); // RV_DFS_SPEED, RV_DFS_CVP
  dfs_param.inType = RV_DFS_IN_V1;
  dfs_param.filterHeight = 4;
  dfs_param.filterWidth = 16;
  dfs_param.disparity.minDisparity = disp_min;
  dfs_param.disparity.numDisparityLevels = disp_lvl;
  dfs_param.doRectification = true;
  dfs_param.ppLevel = rvDFSPPLevel::RV_DFS_PP_SUPREME;

  dfs_param.version = 1;
  dfs_param.inputSize.height = height;
  dfs_param.inputSize.width = width;
  dfs_param.inputSize.stride = stride;

  dfs_param.outputSize.height = height;
  dfs_param.outputSize.width = width;
  dfs_param.outputSize.stride = stride;

  dfs_base = rv_dfs::CreateDFSBase<float>(dfs_param.mode);
  init_flag = dfs_base->initialize(dfs_param, stereo_param);
  if (!init_flag) {
    std::cout << "Not appropriate params. Init failed." << std::endl;
    return;
  }

  rvCameraIntrinsic cm_std = dfs_base->getRectCameraParam().camera[0];
  fx = cm_std.focalLength[0];
  fy = cm_std.focalLength[1];
  cx = cm_std.principalPoint[0];
  cy = cm_std.principalPoint[1];
  // printf("rectified: fx %f, fy %f, cx %f, cy %f\n", fx, fy, cx, cy);

  dispImg = cv::Mat(height, width, CV_32FC1); // grey CV_8UC1
  depImg = cv::Mat(height, width, CV_32FC1);
  dispColor = cv::Mat(height, width, CV_8UC3);
  depShort = cv::Mat(height, width, CV_16UC1);
  rectl = cv::Mat(height, width, CV_8UC1);
  rectr = cv::Mat(height, width, CV_8UC1);
  pc.reserve(pixel);
  initCloudMsg();

  now_img1 = cv::Mat(height, width, CV_8UC1); // in img
  now_img2 = cv::Mat(height, width, CV_8UC1);

  in.meta.version = 0x00010000;
  in.inV1 = DFS_IN_DATA_V1_INIT;

  in.meta.numParams = 1;
  in.meta.paramSize =
      sizeof(rvDFSInputParam); // not zero to enable postprocessing.
  // in.meta.dfsParam = nullptr; // not set roi.
  in.meta.dfsParam = &dfs_run_param;
  in.meta.dfsParam->roi = new rvRoi2D();
  in.meta.dfsParam->roi->x = 0;
  in.meta.dfsParam->roi->y = 0;
  in.meta.dfsParam->roi->width = width;
  in.meta.dfsParam->roi->height = height;

  out.meta.version = 1;
  out.meta.roi = *(in.meta.dfsParam->roi);
  out.meta.dim.width = width;
  out.meta.dim.height = height;
  out.meta.dim.stride = stride;
  out.outV1 = DFS_OUT_DATA_V1_INIT;
  // out.outV1.imgL = nullptr; // hdr image
  // out.outV1.imgR = nullptr;
  out.outV1.mapDataType = 0; // 0 float, 1 int16.

  out.outV1.rectL = rectl.data;
  out.outV1.rectR = rectr.data;
  out.outV1.mapOfDisparity = dispImg.data;
  out.outV1.mapOfDepth = depImg.data;
  out.outV1.pointBuffer = &pc;
  out.outV1.numPoints = pixel;

  disp_id = "disp_color";
  dep_id = "dep_raw";
  left_id = "left_image";
  point_id = "point_cloud";
  rectl_id = "left_rect";
  rectr_id = "right_rect";

  if (stitch_img_flag) {
    stitch_sub = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image", 3,
        std::bind(&DFSRosTwoNode::callbackStitchImg, this, _1));
    t_stereo =
        std::make_shared<std::thread>(&DFSRosTwoNode::processStitchImg, this);
  } else // parse img1 and img2
  {
    img1_sub.subscribe(this, "/camera/infra1/image_rect_raw");
    img2_sub.subscribe(this, "/camera/infra2/image_rect_raw");

    sy.reset(new Sync(MySyncPolicy(5), img1_sub, img2_sub));
    sy->setMaxIntervalDuration(rclcpp::Duration(3, 0));
    sy->registerCallback(
        std::bind(&DFSRosTwoNode::callbackTwoImgs, this, _1, _2));
    t_stereo =
        std::make_shared<std::thread>(&DFSRosTwoNode::processTwoImgs, this);
  }

  if (color_flag) {
    color_pub = image_transport::create_publisher(this, "/dfs_ros2/color_image",
                                                  qos_prf);
    color_sub = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/color/image_raw", 10,
        std::bind(&DFSRosTwoNode::callbackColor, this,
                  _1)); // must not 1. 1 will cause callbackColor dead or stuck
                        // status.
    t_color = std::make_shared<std::thread>(&DFSRosTwoNode::processColor, this);
  }
}

DFSRosTwoNode::~DFSRosTwoNode() {
  if (dfs_base != nullptr)
    dfs_base->deInitialize();
  dfs_base = nullptr;

  if (in.meta.dfsParam != nullptr) {
    if (in.meta.dfsParam->roi != nullptr)
      delete (in.meta.dfsParam->roi);
    if (in.meta.dfsParam->disparity != nullptr)
      delete (in.meta.dfsParam->disparity);
    if (in.meta.dfsParam->ppLevel != nullptr)
      delete (in.meta.dfsParam->ppLevel);

    delete (in.meta.dfsParam);
  }
  in.inV1 = DFS_IN_DATA_V1_INIT; // mat will release by itself.
  out.outV1 = DFS_OUT_DATA_V1_INIT;

  con_stereo.notify_one();
  if (t_stereo && t_stereo->joinable())
    t_stereo->join();

  if (color_flag) {
    con_color.notify_one();
    if (t_color && t_color->joinable())
      t_color->join();
  }
}

void DFSRosTwoNode::parseCameraInfo(const sensor_msgs::msg::CameraInfo &cm1,
                                    const sensor_msgs::msg::CameraInfo &cm2) {
  height = cm1.height;
  width = cm1.width;
  stride = width;
  stereo_param.camera[0].focalLength[0] = cm1.p.at(0);
  stereo_param.camera[0].focalLength[1] = cm1.p.at(5);
  stereo_param.camera[0].principalPoint[0] = cm1.p.at(2);
  stereo_param.camera[0].principalPoint[1] = cm1.p.at(6);
  stereo_param.camera[0].pixelWidth = cm1.width;
  stereo_param.camera[0].pixelHeight = cm1.height;
  stereo_param.camera[0].pixelStride = cm1.width;

  stereo_param.camera[1] = stereo_param.camera[0];

  float fx = cm1.p.at(0);
  float tx = float(cm2.p.at(3) - cm1.p.at(3)) * 1000.0f / fx; // mm unit
  stereo_param.translation[0] = tx;
  stereo_param.translation[1] = 0;
  stereo_param.translation[2] = 0;
  stereo_param.rotation[0] = 0;
  stereo_param.rotation[1] = 0;
  stereo_param.rotation[2] = 0;

  cm_info_flag = true; // fx,fy, cx, cy
  extrin_flag = true;  // t, R
}

void DFSRosTwoNode::processColor() {
  while (rclcpp::ok()) {
    std::unique_lock<std::mutex> lk2(mtx_color);
    con_color.wait(lk2, [&] { return get_color == true; });
    cv::Mat one = cv_p3->image.clone();
    std_msgs::msg::Header head_one = cv_p3->header;
    get_color = false;
    lk2.unlock();

    sensor_msgs::msg::Image::SharedPtr one_msg =
        cv_bridge::CvImage(head_one, enc::BGR8, one).toImageMsg();
    color_pub.publish(*one_msg);
  }
}

void DFSRosTwoNode::callbackColor(
    const sensor_msgs::msg::Image::SharedPtr color_p) // must no & reference.
{
  mtx_color.lock();
  cv_p3 = cv_bridge::toCvCopy(color_p, enc::BGR8);
  get_color = true;
  mtx_color.unlock();
  con_color.notify_one();
}

void DFSRosTwoNode::callbackTwoImgs(
    const sensor_msgs::msg::Image::ConstSharedPtr &img_p1,
    const sensor_msgs::msg::Image::ConstSharedPtr &img_p2) {
  mtx_stereo.lock();
  cv_p1 = cv_bridge::toCvCopy(img_p1,
                              enc::MONO8); // alloc mem and copy data to cv_p1
  cv_p2 = cv_bridge::toCvCopy(img_p2, enc::MONO8);
  get_stereo = true;
  mtx_stereo.unlock();
  con_stereo.notify_one();
}

void DFSRosTwoNode::processTwoImgs() {
  while (rclcpp::ok()) {
    start = std::chrono::high_resolution_clock::now();

    std::unique_lock<std::mutex> lk(mtx_stereo);
    con_stereo.wait(lk, [&] { return get_stereo == true; });
    now_img1 = cv_p1->image.clone(); // cv::Mat
    now_img2 = cv_p2->image.clone();
    img_head = cv_p1->header;
    get_stereo = false;
    lk.unlock();

    in.inV1.imgLeft = now_img1.ptr<uint8_t>();
    in.inV1.imgRight = now_img2.ptr<uint8_t>();

    dfs_base->compute(&in, &out);

    updateCloudMsg();
    toColorByUchar(dispImg, dispColor);
    toShort16(depImg, depShort);
    // if( dfs_cnt<3) debug_img();
    sensor_msgs::msg::Image::SharedPtr left_msg =
        cv_bridge::CvImage(genHeader(left_id), enc::MONO8, now_img1)
            .toImageMsg();
    sensor_msgs::msg::Image::SharedPtr disp_msg =
        cv_bridge::CvImage(genHeader(disp_id), enc::BGR8, dispColor)
            .toImageMsg();
    sensor_msgs::msg::Image::SharedPtr dep_msg =
        cv_bridge::CvImage(genHeader(dep_id), enc::TYPE_16UC1, depShort)
            .toImageMsg();
    sensor_msgs::msg::Image::SharedPtr rectl_msg =
        cv_bridge::CvImage(genHeader(rectl_id), enc::MONO8, rectl).toImageMsg();
    sensor_msgs::msg::Image::SharedPtr rectr_msg =
        cv_bridge::CvImage(genHeader(rectr_id), enc::MONO8, rectr).toImageMsg();

    left_pub.publish(*left_msg);
    disp_pub.publish(*disp_msg);
    dep_pub.publish(*dep_msg);
    rectl_pub.publish(*rectl_msg);
    rectr_pub.publish(*rectr_msg);
    point_pub->publish(cloud_msg);

    // dfs_cnt++;
    stop = std::chrono::high_resolution_clock::now();
    elapsed = stop - start;
    std::cout << "[INFO] Time(ms) of calcauteAll is: " << elapsed.count() * 1000
              << std::endl;
  }
}

void DFSRosTwoNode::callbackStitchImg(
    const sensor_msgs::msg::Image::SharedPtr img_p1) {
  mtx_stereo.lock();
  cv_p1 = cv_bridge::toCvCopy(img_p1,
                              enc::MONO8); // alloc mem and copy data to cv_p1
  get_stereo = true;
  mtx_stereo.unlock();
  con_stereo.notify_one();
}

void DFSRosTwoNode::processStitchImg() {
  cv::Mat left_img;
  while (rclcpp::ok()) {
    start = std::chrono::high_resolution_clock::now();

    std::unique_lock<std::mutex> lk(mtx_stereo);
    con_stereo.wait(lk, [&] { return get_stereo == true; });
    now_img1 = cv_p1->image.clone(); // cv::Mat
    img_head = cv_p1->header;
    get_stereo = false;
    lk.unlock();

    in.inV1.imgLeft = now_img1.ptr<uint8_t>(); // left-and-right img
    dfs_base->compute(&in, &out);

    updateCloudMsg();
    left_img = now_img1(cv::Rect(0, 0, width, height)).clone();
    toColorByUchar(dispImg, dispColor);
    toShort16(depImg, depShort);
    // if( dfs_cnt<3) debug_img();
    sensor_msgs::msg::Image::SharedPtr left_msg =
        cv_bridge::CvImage(genHeader(left_id), enc::MONO8, left_img)
            .toImageMsg();
    sensor_msgs::msg::Image::SharedPtr disp_msg =
        cv_bridge::CvImage(genHeader(disp_id), enc::BGR8, dispColor)
            .toImageMsg();
    sensor_msgs::msg::Image::SharedPtr dep_msg =
        cv_bridge::CvImage(genHeader(dep_id), enc::TYPE_16UC1, depShort)
            .toImageMsg();
    sensor_msgs::msg::Image::SharedPtr rectl_msg =
        cv_bridge::CvImage(genHeader(rectl_id), enc::MONO8, rectl).toImageMsg();
    sensor_msgs::msg::Image::SharedPtr rectr_msg =
        cv_bridge::CvImage(genHeader(rectr_id), enc::MONO8, rectr).toImageMsg();

    left_pub.publish(*left_msg);
    disp_pub.publish(*disp_msg);
    dep_pub.publish(*dep_msg);
    rectl_pub.publish(*rectl_msg);
    rectr_pub.publish(*rectr_msg);
    point_pub->publish(cloud_msg);

    // dfs_cnt++;
    stop = std::chrono::high_resolution_clock::now();
    elapsed = stop - start;
    std::cout << "[INFO] Time(ms) of calcauteAll is: " << elapsed.count() * 1000
              << std::endl;
  }
}

std_msgs::msg::Header DFSRosTwoNode::genHeader(const std::string &frame_id) {
  std_msgs::msg::Header head; // no member seq
  head.stamp = img_head.stamp;
  head.frame_id = frame_id;
  return head;
}

// sensor_msgs::msg::PointCloud2 DFSRosTwoNode::genPointCloudMsg(const
// std::string& frame_id)
// {
//     sensor_msgs::msg::PointCloud2 cloud;
//     cloud_msg.header = genHeader(frame_id);
//     cloud_msg.height = 1;
//     cloud_msg.width = pc.size();

//     for(int i=0; i<pc.size(); i++){
//         pc[i][0] = pc[i][0]/1000.0f;
//         pc[i][1] = pc[i][1]/1000.0f;
//         pc[i][2] = pc[i][2]/1000.0f;
//     }
//     cloud_msg.fields.resize( 3);
//     cloud_msg.fields[0].name = "x";
//     cloud_msg.fields[0].offset =0;
//     cloud_msg.fields[0].datatype = 7;  // FLOAT32
//     cloud_msg.fields[0].count = 1;
//     cloud_msg.fields[1].name = "y";
//     cloud_msg.fields[1].offset =4;
//     cloud_msg.fields[1].datatype = 7;
//     cloud_msg.fields[1].count = 1;
//     cloud_msg.fields[2].name = "z";
//     cloud_msg.fields[2].offset =8;
//     cloud_msg.fields[2].datatype = 7;
//     cloud_msg.fields[2].count = 1;

//     cloud_msg.is_bigendian = false;
//     cloud_msg.point_step = 16;
//     cloud_msg.row_step = cloud_msg.point_step* pc.size();
//     cloud_msg.data.resize( cloud_msg.row_step );
//     for(int i=0; i<pc.size(); i++){
//         memcpy( &cloud_msg.data[i*cloud_msg.point_step], pc[i].data(), 12);
//         memset( &cloud_msg.data[i*cloud_msg.point_step + 12], 0, 4);
//     }
//     cloud_msg.is_dense = true;
//     return cloud;
// }

// sensor_msgs::msg::PointCloud2 DFSRosTwoNode::genPointCloudColorMsg(const
// std::string& frame_id)
// {
//     sensor_msgs::msg::PointCloud2 cloud;
//     cloud_msg.header = genHeader(frame_id);
//     cloud_msg.height = 1;
//     cloud_msg.width = pcc.size();

//     for(int i=0; i<pcc.size(); i++){
//         pcc[i][0] = pcc[i][0]/1000.0f;
//         pcc[i][1] = pcc[i][1]/1000.0f;
//         pcc[i][2] = pcc[i][2]/1000.0f;
//     }

//     cloud_msg.fields.resize( 4);
//     cloud_msg.fields[0].name = "x";
//     cloud_msg.fields[0].offset =0;
//     cloud_msg.fields[0].datatype = 7;  // FLOAT32
//     cloud_msg.fields[0].count = 1;
//     cloud_msg.fields[1].name = "y";
//     cloud_msg.fields[1].offset =4;
//     cloud_msg.fields[1].datatype = 7;
//     cloud_msg.fields[1].count = 1;
//     cloud_msg.fields[2].name = "z";
//     cloud_msg.fields[2].offset =8;
//     cloud_msg.fields[2].datatype = 7;
//     cloud_msg.fields[2].count = 1;

//     cloud_msg.fields[3].name = "rgb";
//     cloud_msg.fields[3].offset =16;
//     cloud_msg.fields[3].datatype = 7;
//     cloud_msg.fields[3].count = 1;

//     cloud_msg.is_bigendian = false;
//     cloud_msg.point_step = 20;
//     cloud_msg.row_step = cloud_msg.point_step* pcc.size();
//     cloud_msg.data.resize( cloud_msg.row_step);

//     int offset = 0;
//     for(int i=0; i<pcc.size(); i++){
//         offset = i* cloud_msg.point_step;
//         memcpy(&cloud_msg.data[offset], pcc[i].data(), 12);
//         memset(&cloud_msg.data[offset + 12], 0, 4);
//         cloud_msg.data[offset + 16] = static_cast<unsigned char>(pcc[i][3]);
//         cloud_msg.data[offset + 17] = static_cast<unsigned char>(pcc[i][4]);
//         cloud_msg.data[offset + 18] = static_cast<unsigned char>(pcc[i][5]);
//         cloud_msg.data[offset + 19] = 0;
//     }
//     cloud_msg.is_dense = true;
//     return cloud;
// }

void DFSRosTwoNode::initCloudMsg() {
  cloud_msg.header.frame_id = "point_cloud";
  cloud_msg.height = 1;
  cloud_msg.width = pixel;

  cloud_msg.is_bigendian = false;
  cloud_msg.is_dense = true;

  if (color_point_flag) {
    cloud_msg.fields.resize(4);
    cloud_msg.point_step = 20;

    cloud_msg.fields[3].name = "rgb";
    cloud_msg.fields[3].offset = 16;
    cloud_msg.fields[3].datatype = 7;
    cloud_msg.fields[3].count = 1;
  } else {
    cloud_msg.fields.resize(3);
    cloud_msg.point_step = 16;
  }

  cloud_msg.row_step = cloud_msg.point_step * pixel; // pc.size()
  cloud_msg.data.resize(cloud_msg.row_step);

  cloud_msg.fields[0].name = "x";
  cloud_msg.fields[0].offset = 0;
  cloud_msg.fields[0].datatype = 7; // FLOAT32
  cloud_msg.fields[0].count = 1;
  cloud_msg.fields[1].name = "y";
  cloud_msg.fields[1].offset = 4;
  cloud_msg.fields[1].datatype = 7;
  cloud_msg.fields[1].count = 1;
  cloud_msg.fields[2].name = "z";
  cloud_msg.fields[2].offset = 8;
  cloud_msg.fields[2].datatype = 7;
  cloud_msg.fields[2].count = 1;
}

void DFSRosTwoNode::updateCloudMsg() {
  cloud_msg.header.stamp = img_head.stamp; // time class
  cloud_msg.width = pc.size();
  cloud_msg.row_step = cloud_msg.point_step * pc.size();
  cloud_msg.data.resize(cloud_msg.row_step);

  for (int i = 0; i < pc.size(); i++) { // point cloud msg use meter unit.
    pc[i][0] = pc[i][0] / 1000.0f;      // (0.2, 0.035, 0.107)
    pc[i][1] = pc[i][1] / 1000.0f;
    pc[i][2] = pc[i][2] / 1000.0f;
  }

  if (color_point_flag) // xyzrgb
  {
    int offset = 0;
    int ii = 0;
    uint8_t *p1 = rectl.data;
    if (rectl.channels() == 1) { // xyzggg
      for (int i = 0; i < pc.size(); i++) {
        ii = width * (int)(cy - pc[i][2] * fy / pc[i][0]) +
             (int)(cx - pc[i][1] * fx / pc[i][0] + 0.5f); // hwc
        offset = i * cloud_msg.point_step;
        memcpy(&cloud_msg.data[offset], pc[i].data(), 12);
        memset(&cloud_msg.data[offset + 12], 0, 4);
        cloud_msg.data[offset + 16] = p1[ii];
        cloud_msg.data[offset + 17] = p1[ii];
        cloud_msg.data[offset + 18] = p1[ii];
        cloud_msg.data[offset + 19] = 0;
      }
    } else { // xyzrgb
      for (int i = 0; i < pc.size(); i++) {
        ii = 3 * (width * (int)(cy - pc[i][2] * fy / pc[i][0]) +
                  (int)(cx - pc[i][1] * fx / pc[i][0] + 0.5f)); // hwc
        offset = i * cloud_msg.point_step;
        memcpy(&cloud_msg.data[offset], pc[i].data(), 12);
        memset(&cloud_msg.data[offset + 12], 0, 4);
        cloud_msg.data[offset + 16] = p1[ii];
        cloud_msg.data[offset + 17] = p1[ii + 1];
        cloud_msg.data[offset + 18] = p1[ii + 2];
        cloud_msg.data[offset + 19] = 0;
      }
    }
  } else { // xyz
    for (int i = 0; i < pc.size(); i++) {
      memcpy(&cloud_msg.data[i * cloud_msg.point_step], pc[i].data(), 12);
      memset(&cloud_msg.data[i * cloud_msg.point_step + 12], 0, 4);
    }
  }
}

void DFSRosTwoNode::toShort16(cv::Mat &fmat, cv::Mat &smat) {
  cv::resize(smat, smat, fmat.size(), 0, 0, cv::INTER_LINEAR);
  unsigned short *sp = (unsigned short *)smat.data;
  float *fp = fmat.ptr<float>();
  for (int ii = 0; ii < fmat.cols * fmat.rows; ++ii)
    sp[ii] = static_cast<unsigned short>(round(fp[ii]));
}

void DFSRosTwoNode::toColorByUchar(cv::Mat &fmat, cv::Mat &umat) {
  cv::Mat mid_mat(fmat.size(), CV_8UC1);
  unsigned char *mp = (unsigned char *)mid_mat.data;
  float *fp = fmat.ptr<float>();
  for (int ii = 0; ii < fmat.cols * fmat.rows; ++ii)
    mp[ii] = static_cast<unsigned char>(round(fp[ii]));
  getMinMaxThenColor(mid_mat, umat);
}

void DFSRosTwoNode::debug_info(const std::string &s) {
  std::cout << "--------------------------------------------------" << s
            << std::endl;
  std::cout << "height " << height << ", width " << width << ", stride "
            << stride << std::endl;

  const rvStereoCamera &param = stereo_param;
  std::cout << "---- Stereo Param Info ----" << std::endl;
  std::cout << "translation param: " << param.translation[0] << ","
            << param.translation[1] << "," << param.translation[2] << std::endl;
  std::cout << "rotation param: " << param.rotation[0] << ","
            << param.rotation[1] << "," << param.rotation[2] << std::endl;

  std::cout << "camera 1 param: " << std::endl;
  std::cout << "camera width, height, stride: " << param.camera[0].pixelWidth
            << "," << param.camera[0].pixelHeight << ", "
            << param.camera[0].pixelStride << std::endl;
  std::cout << "focal length: " << param.camera[0].focalLength[0] << ","
            << param.camera[0].focalLength[1] << std::endl;
  std::cout << "principal point: " << param.camera[0].principalPoint[0] << ","
            << param.camera[0].principalPoint[1] << std::endl;
  std::cout << "distortionModel: " << param.camera[0].distortionModel
            << std::endl;
  std::cout << "distortion:  " << param.camera[0].distortion[0] << ", "
            << param.camera[0].distortion[1];
  std::cout << "," << param.camera[0].distortion[2] << ", "
            << param.camera[0].distortion[3];
  std::cout << "," << param.camera[0].distortion[4] << ", "
            << param.camera[0].distortion[5];
  std::cout << "," << param.camera[0].distortion[6] << ", "
            << param.camera[0].distortion[7];
  std::cout << std::endl;

  std::cout << "camera 2 param: " << std::endl;
  std::cout << "camera width, height, stride: " << param.camera[1].pixelWidth
            << "," << param.camera[1].pixelHeight << ", "
            << param.camera[1].pixelStride << std::endl;
  std::cout << "focal length: " << param.camera[1].focalLength[0] << ","
            << param.camera[1].focalLength[1] << std::endl;
  std::cout << "principal point: " << param.camera[1].principalPoint[0] << ","
            << param.camera[1].principalPoint[1] << std::endl;
  std::cout << "distortionModel: " << param.camera[1].distortionModel
            << std::endl;
  std::cout << "distortion:  " << param.camera[1].distortion[0] << ", "
            << param.camera[1].distortion[1];
  std::cout << "," << param.camera[1].distortion[2] << ", "
            << param.camera[1].distortion[3];
  std::cout << "," << param.camera[1].distortion[4] << ", "
            << param.camera[1].distortion[5];
  std::cout << "," << param.camera[1].distortion[6] << ", "
            << param.camera[1].distortion[7];
  std::cout << std::endl;
  std::cout << "--------------------------------------------------"
            << std::endl;
}

void DFSRosTwoNode::debug_img() {
  std::string sfx = std::to_string(dfs_cnt) + ".png";
  std::string pc_sfx = std::to_string(dfs_cnt) + ".ply";
  if (!now_img1.empty())
    cv::imwrite(dbg_dir + "/left_" + sfx, now_img1);
  if (!now_img2.empty())
    cv::imwrite(dbg_dir + "/right_" + sfx, now_img2);
  if (!dispColor.empty())
    cv::imwrite(dbg_dir + "/disp_" + sfx, dispColor);
  if (!depShort.empty())
    cv::imwrite(dbg_dir + "/dep_" + sfx, depShort);
  if (!dispImg.empty())
    cv::imwrite(dbg_dir + "/dispRaw_" + sfx, dispImg);

  if (pc.size())
    writePLYPointCloud(dbg_dir + "/pc_" + pc_sfx, pc);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr gnode = std::make_shared<DFSRosTwoNode>(my_qos);

  gnode->declare_parameter<int>("dfs_mode", 0);
  gnode->declare_parameter<int>("disparity_min", 0);
  gnode->declare_parameter<int>("disparity_level", 0);
  gnode->declare_parameter<bool>("enable_color", false);
  gnode->declare_parameter<bool>("enable_calib_file", false);
  gnode->declare_parameter<bool>("enable_stitch_img", false);
  gnode->declare_parameter<bool>("enable_color_point", false);
  gnode->declare_parameter<std::string>("calib_file_path", "");

  std::shared_ptr<DFSRosTwoNode> cnode =
      std::dynamic_pointer_cast<DFSRosTwoNode>(gnode);
  cnode->init_dfs();

  rclcpp::spin(gnode);
  printf("dfs ros2 service.");
  return 0;
}