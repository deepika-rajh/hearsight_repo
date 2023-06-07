/*****************************************************************************
@copyright
Copyright (c) 2020-2023 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef __STDC_FORMAT_MACROS
#define __STDC_FORMAT_MACROS
#endif
#include <inttypes.h>

#include <thread>
#include <signal.h>
#include <functional>
#include <string.h>
#include <math.h>

#include "VMSystem.h"
#include "SystemTime.h"

#include <sstream>
#include <fstream>
#include <cassert>

#include <rvQueue.h>
#include <opencv2/opencv.hpp>

#ifdef ROS_BASED
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#endif

queue_mt<sensor_hijack> hijackArray(BUF_SIZE);

//static members definition
rvVM* VMSystem::vmPtr = nullptr;
#if 0
rvDFS* VMSystem::dfsPtr = nullptr;
uint8_t* VMSystem::lImage = nullptr;
uint8_t* VMSystem::rImage = nullptr;
float* VMSystem::depthImageF = nullptr;
uint16_t* VMSystem::depthImageS = nullptr;
#endif
std::shared_ptr<VMSystem> VMSystem::t = nullptr;
std::shared_ptr<VSLAMHijack> VMSystem::hijack = nullptr;
VMSystem::SystemState VMSystem::systemState = KSLEEPING;
std::string VMSystem::algConfFile = "";
rvCameraParams VMSystem::cameraConfiguration;
std::shared_ptr<CameraInterface> VMSystem::inputCamera = nullptr;

int VMSystem::width = 0;
int VMSystem::height = 0;


/************************************** C APIs start ************/
void Euler2Quaternion(double roll, double pitch, double yaw, double quaternion[4])
{
    double t0 = cos(yaw * 0.5);
    double t1 = sin(yaw * 0.5);
    double t2 = cos(roll * 0.5);
    double t3 = sin(roll * 0.5);
    double t4 = cos(pitch * 0.5);
    double t5 = sin(pitch * 0.5);

    quaternion[0] = t0 * t2 * t4 + t1 * t3 * t5;  //w
    quaternion[1] = t0 * t3 * t4 - t1 * t2 * t5;  //x
    quaternion[2] = t0 * t2 * t5 + t1 * t3 * t4;  //y
    quaternion[3] = t1 * t2 * t4 - t0 * t3 * t5;  //z
}

#ifdef ROS_BASED
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
using std::placeholders::_1;

extern rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr raw_pose_pub;
extern rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr robot_pose_pub;
extern rclcpp::Node::SharedPtr g_node;

extern image_transport::Publisher    occupancy_img_pub;

void pub_camera_raw_pose(const rvVSLAMPose& pose)
{
    auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();

    odom_msg->header.frame_id = "odom";
    odom_msg->child_frame_id = "base_link";
    odom_msg->header.stamp = rclcpp::Time(pose.timestampNs, RCL_ROS_TIME);

    odom_msg->pose.pose.position.x = pose.pose.translation[0];
    odom_msg->pose.pose.position.y = pose.pose.translation[1];
    odom_msg->pose.pose.position.z = pose.pose.translation[2];

    double q[4];
    Euler2Quaternion(pose.pose.euler[0], pose.pose.euler[1], pose.pose.euler[2], q);

    odom_msg->pose.pose.orientation.x = q[1];
    odom_msg->pose.pose.orientation.y = q[2];
    odom_msg->pose.pose.orientation.z = q[3];
    odom_msg->pose.pose.orientation.w = q[0];

    odom_msg->twist.twist.linear.x = 0;
    odom_msg->twist.twist.angular.z = 0;

    raw_pose_pub->publish(std::move(odom_msg));
}

void pub_robot_pose(const rvVSLAMPose& pose)
{
    auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();

    odom_msg->header.frame_id = "odom";
    odom_msg->child_frame_id = "base_link";
    odom_msg->header.stamp = rclcpp::Time(pose.timestampNs, RCL_ROS_TIME);

    odom_msg->pose.pose.position.x = pose.pose.translation[0];
    odom_msg->pose.pose.position.y = pose.pose.translation[1];
    odom_msg->pose.pose.position.z = pose.pose.translation[2];

    double q[4];
    Euler2Quaternion(pose.pose.euler[0], pose.pose.euler[1], pose.pose.euler[2], q);

    odom_msg->pose.pose.orientation.x = q[1];
    odom_msg->pose.pose.orientation.y = q[2];
    odom_msg->pose.pose.orientation.z = q[3];
    odom_msg->pose.pose.orientation.w = q[0];

    odom_msg->twist.twist.linear.x = 0;
    odom_msg->twist.twist.angular.z = 0;

    robot_pose_pub->publish(std::move(odom_msg));
}

void VMSystem::state_callbackROS(const std_msgs::msg::String::SharedPtr msg) const
{
    state_callback(msg->data);
}


void Quaternion2Matrix( quaternion_type *q, double *m )
{
   m[0] = 2 * q->w * q->w + 2 * q->x * q->x - 1;
   m[1] = 2 * q->x * q->y - 2 * q->w * q->z;
   m[2] = 2 * q->x * q->z + 2 * q->w * q->y;
   m[3] = 2 * q->x * q->y + 2 * q->w * q->z;
   m[4] = 2 * q->w * q->w + 2 * q->y * q->y - 1;
   m[5] = 2 * q->y * q->z - 2 * q->w * q->x;
   m[6] = 2 * q->x * q->z - 2 * q->w * q->y;
   m[7] = 2 * q->y * q->z + 2 * q->w * q->x;
   m[8] = 2 * q->w * q->w + 2 * q->z * q->z - 1;
}


void VMSystem::pose_callbackROS(const nav_msgs::msg::Odometry::SharedPtr msg) const
{
    rvPose6DRTWithTimestamp cameraPose;

    quaternion_type q;
    q.x = msg->pose.pose.orientation.x;
    q.y = msg->pose.pose.orientation.y;
    q.z = msg->pose.pose.orientation.z;
    q.w = msg->pose.pose.orientation.w;
    double rotation[9];
    Quaternion2Matrix(&q, rotation);
    cameraPose.pose.matrix[0][0] = rotation[0];
    cameraPose.pose.matrix[0][1] = rotation[1];
    cameraPose.pose.matrix[0][2] = rotation[2];
    cameraPose.pose.matrix[1][0] = rotation[3];
    cameraPose.pose.matrix[1][1] = rotation[4];
    cameraPose.pose.matrix[1][2] = rotation[5];
    cameraPose.pose.matrix[2][1] = rotation[6];
    cameraPose.pose.matrix[2][2] = rotation[7];
    cameraPose.pose.matrix[2][3] = rotation[8];

    cameraPose.pose.matrix[0][3] = msg->pose.pose.position.x;
    cameraPose.pose.matrix[1][3] = msg->pose.pose.position.y;
    cameraPose.pose.matrix[2][3] = msg->pose.pose.position.z; 

    cameraPose.timestamp  = rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec, RCL_ROS_TIME).nanoseconds();

    addCameraPose(cameraPose);

    printf("Input Camera pose time is %ld\n", cameraPose.timestamp);
}
#endif

/**********************   C APIs end   ************************************/

VMSystem::~VMSystem()
{
}


void VMSystem::deinit()
{
    printf("VM de-initializing");

#if 0
    if (dfsPtr)
    {
       rvDFS_Deinitialize(dfsPtr);
       delete[] lImage;
       delete[] rImage;
       delete[] depthImageF;
       delete[] depthImageS;
    }
#endif

    if (vmPtr)
       rvVM_Deinitialize(vmPtr);

    inputCamera = nullptr;
    hijack = nullptr;
    vmPtr = nullptr;
}


VMSystem::VMSystem(std::shared_ptr<CameraInterface>& camera)
{
    systemState = KSLEEPING;

    inputCamera = camera;
    if (inputCamera)
    {
        inputCamera->addCallback(addDepthImage);
        inputCamera->start();
        cameraConfiguration = inputCamera->getCameraConfiguration();
        //inputCamera->stop();
    }

#ifdef ROS_BASED
    cameraInMapPose_sub = g_node->create_subscription<nav_msgs::msg::Odometry>("vslam_odom_raw", 10,
        std::bind(&VMSystem::pose_callbackROS, this, _1));
    state_sub = g_node->create_subscription<std_msgs::msg::String>("VM_state", 10,
        std::bind(&VMSystem::state_callbackROS, this, _1));
#endif
}

void EulerToSO3_1(const float32_t* euler, float32_t* rotation)
{
    float32_t cr = (float32_t)cos(euler[0]);
    float32_t sr = (float32_t)sin(euler[0]);
    float32_t cp = (float32_t)cos(euler[1]);
    float32_t sp = (float32_t)sin(euler[1]);
    float32_t cy = (float32_t)cos(euler[2]);
    float32_t sy = (float32_t)sin(euler[2]);
    rotation[0 * 3 + 0] = cy * cp;
    rotation[0 * 3 + 1] = cy * sp * sr - sy * cr;
    rotation[0 * 3 + 2] = cy * sp * cr + sy * sr;
    rotation[1 * 3 + 0] = sy * cp;
    rotation[1 * 3 + 1] = sy * sp * sr + cy * cr;
    rotation[1 * 3 + 2] = sy * sp * cr - cy * sr;
    rotation[2 * 3 + 0] = -sp;
    rotation[2 * 3 + 1] = cp * sr;
    rotation[2 * 3 + 2] = cp * cr;
}


std::shared_ptr<VMSystem> VMSystem::Initialize(const std::string& algSetting, std::shared_ptr<CameraInterface> camera, bool _showImg)
{
    algConfFile = algSetting;

    if (t.get() == nullptr)
    {
        t = std::make_shared<VMSystem>(camera);
        cameraConfiguration = inputCamera->getCameraConfiguration();
        width = cameraConfiguration.stereo.camera[0].pixelWidth;
        height = cameraConfiguration.stereo.camera[0].pixelHeight;

        hijack = std::make_shared<VSLAMHijack>();
        if (hijack)
        {
            std::shared_ptr<HijackReceiver> tmp = t;
            hijack->addReceiver(tmp);
        }

#if 0
        if (cameraConfiguration.cameraType == rvStereo)
        {
           int gMinDisparity = 1;
           int gLevelDisparity = 96;
           rvDFSParameter dfs_parameter;
           dfs_parameter.mode = RV_DFS_SPEED;
           dfs_parameter.filterHeight = 11;
           dfs_parameter.filterWidth = 15;
           dfs_parameter.disparity.minDisparity = gMinDisparity;
           dfs_parameter.disparity.numDisparityLevels = gLevelDisparity;
           dfs_parameter.doRectification = true;

           dfsPtr = rvDFS_Initialize(cameraConfiguration.stereo.camera[0].pixelWidth,
              cameraConfiguration.stereo.camera[0].pixelHeight, cameraConfiguration.stereo.camera[0].pixelStride,
              dfs_parameter, cameraConfiguration.stereo);
           int pixelNum = cameraConfiguration.stereo.camera[0].pixelWidth * cameraConfiguration.stereo.camera[0].pixelHeight;
           lImage = new uint8_t[pixelNum];
           rImage = new uint8_t[pixelNum];
           depthImageF = new float[pixelNum];
           depthImageS = new uint16_t[pixelNum];


           rvStereoCamera rectCamera = rvDFS_GetRectifiedCameraParameter(dfsPtr);
           vmPtr = rvVM_Initialize(&rectCamera.camera[0], algConfFile.c_str());
        }
        else
#endif
        {
           vmPtr = rvVM_Initialize(&(cameraConfiguration.stereo.camera[0]), algConfFile.c_str());
        }

#if GDB_DEBUG  //SIGINT would go to gdb but not VM application
        signal(48, Stop);
#else
        signal(SIGINT, Stop);
#endif
    }

    return t;
}

void VMSystem::Run()
{
    if (systemState == KSTOPPING)
    {
        return;
    }
    if (hijack)
        hijack->start();

    rvVM_Run(vmPtr);
    systemState = KWORKING;
}

void VMSystem::sleep()
{
    if (systemState == KSLEEPING)
        return;

    systemState = KSLEEPING;
    rvVM_Sleep(vmPtr);
}

void VMSystem::awake()
{
    if (systemState == KSLEEPING)
    {
        systemState = KWORKING;
        rvVM_Awake(vmPtr);
    }
}

void VMSystem::restart()
{
    rvVM_Restart(vmPtr);
}


void VMSystem::addDepthImage(const int64_t timestamp, const uint8_t* imageBuf, const uint16_t* depthBufInput)
{
    if (VMSystem::systemState == KSLEEPING)
        return;

    const uint16_t* depthBuf;
#if 0
    if (depthBufInput == NULL && dfsPtr != nullptr)
    {
       int pixelNum = cameraConfiguration.stereo.camera[0].pixelStride * cameraConfiguration.stereo.camera[0].pixelHeight;
       memcpy(lImage, imageBuf, pixelNum);
       memcpy(rImage, imageBuf + pixelNum, pixelNum);
       rvDFS_CalculateDepth(dfsPtr, lImage, rImage, depthImageF);
       for (size_t i = 0; i < pixelNum; i++)
          depthImageS[i] = (uint16_t)(depthImageF[i]);
       cv::Mat depthMat(cameraConfiguration.stereo.camera[0].pixelHeight, cameraConfiguration.stereo.camera[0].pixelStride, CV_16UC1, depthImageS);
       depthBuf = depthImageS;
    }
    else
#endif
    {
       depthBuf = depthBufInput;
    }

    rvVM_AddOneImage(vmPtr, depthBuf, timestamp);

    /*************FOR USERS:***************************************************/
    /********sample codes to show depth image for debugging purpose************/
    int depthCutoff = 6000; //mm
    cv::Mat depthImage8Bit(height, width, CV_8UC1);
    for (int ii = 0; ii < width * height; ++ii)
    {
        if (depthBuf[ii] > depthCutoff)
            depthImage8Bit.data[ii] = 0;
        else if (depthBuf[ii] == 0) // invalid value
            depthImage8Bit.data[ii] = 0;
        else
            depthImage8Bit.data[ii] = 255 - (int) (depthBuf[ii] * 255 / depthCutoff);        
    }
#ifndef __linux__
    cv::imshow("depth image", depthImage8Bit);
    cv::waitKey(1);
#endif
    /************finish showing depth image************************************/

    /*************FOR USERS:***************************************************/
    /*************sample codes to get grid map*********************************/
    /*************these codes can be moved to any other function/thread********/
    unsigned char* gridMap = NULL;
    int gridWidth;
    int gridHeight;
    int gridOriginX;
    int gridOriginY;
    int64_t gridTimestamp;
    if (rvVM_GetGridMap(vmPtr, &gridMap, &gridWidth, &gridHeight, &gridOriginX, &gridOriginY, &gridTimestamp))
    {
        assert(gridWidth > 0 && gridHeight > 0 && gridMap != NULL);
        //add code here to handle grid map
        cv::Mat gridImage(gridHeight, gridWidth, CV_8UC1);
        memcpy(gridImage.data, gridMap, gridHeight * gridWidth);
        delete[]gridMap;
        gridMap = NULL;
#ifndef __linux__
        cv::imshow("grid map", gridImage);
        cv::waitKey(1);
#endif
#ifdef ROS_BASED
	  sensor_msgs::msg::Image::SharedPtr img;
      img = cv_bridge::CvImage(
         std_msgs::msg::Header(), sensor_msgs::image_encodings::MONO8, gridImage ).toImageMsg();

      rclcpp::Clock ros_clock( RCL_ROS_TIME );

      img->width = gridWidth;
      img->height = gridHeight;
      img->is_bigendian = false;
      img->step = gridWidth;
      img->header.frame_id = "occupancy_image";
      img->header.stamp = ros_clock.now();
      occupancy_img_pub.publish( img );
#endif
    }
    /************finish getting grid map***************************************/

    /*************FOR USERS:***************************************************/
    /*************sample codes to get voxel map********************************/
    /*************these codes can be moved to any other function/thread********/
    float32_t* voxelPoints = NULL;
    int64_t voxelTimestamp;
    int numVoxelPoints = rvVM_GetVoxelMap(vmPtr, &voxelPoints, &voxelTimestamp);
    if (numVoxelPoints > 0)
    {
        //add code here to handle voxels (point cloud)
        delete[]voxelPoints;
        voxelPoints = NULL;
    }
    /************finish getting voxel map***************************************/

    /*************FOR USERS:****************************************************/
    /*************sample codes to save grid map*********************************/
    /*************these codes can be moved to any other function/thread*********/
    // if (timestamp == 1607411709209444000)
    {
        bool saveFlag = rvVM_SaveGridMap(vmPtr);
    }
    /************finish saving grid map*****************************************/
}


void VMSystem::addHijack(bool status, int64_t timestamp)
{
    /***********FOR USERS:***************************************************/
    /***********Any operation after a detected hijack can be added here******/
}


void VMSystem::Spin()
{
#ifdef ROS_BASED
    rclcpp::spin(g_node);
#elif defined (ROS1_BASED)
    ros::spin();
#else 

    while (systemState != KSTOPPING)
    {
        VSLAM_SLEEP(10);
#ifdef SIMULATION
        rvVSLAMPose  rawPose = rvVWSLAM_GetVslamRawPose(vslamPtr);
        {
            std::lock_guard<std::mutex> lk(mut);
            rawPoseTimeStamp = rawPose.timestampNs;
            data_cond.notify_all();
        }
#endif //SIMULATION
    }
#ifdef SIMULATION
    {
        std::lock_guard<std::mutex> lk(mut);
        rawPoseTimeStamp = currentImageTimeStamp;
        data_cond.notify_all();
    }
#endif //SIMULATION
#endif
}

void VMSystem::state_callback(const std::string& msg)
{
    printf("vm received state change msg: %s\n", msg.c_str());

    if (!strcmp(msg.c_str(), "stop"))
    {
        printf("vm received stop sig\n");
        Stop(SIGINT);
    }
    else if (!strcmp(msg.c_str(), "sleep")) {
        printf("vm received sleep sig\n");
        sleep();
    }
    else if (!strcmp(msg.c_str(), "awake")) {
        printf("vm received awake sig\n");
        awake();
    }
    else if (!strcmp(msg.c_str(), "restart")) {
        printf("vm received restart sig\n");
        restart();
    }
}

void VMSystem::Quit(void)
{
    rvVM_Stop(vmPtr);

    if (hijack)
        hijack->stop();

    if (inputCamera)
        inputCamera->stop();
}


void VMSystem::addCameraPose(const rvPose6DRTWithTimestamp& pose)
{
    rvVM_AddOnePose(vmPtr, pose.pose, pose.timestamp);
}
