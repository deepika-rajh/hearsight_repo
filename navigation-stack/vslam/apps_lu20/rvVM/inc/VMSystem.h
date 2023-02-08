/*****************************************************************************
@copyright
Copyright (c) 2020-2023 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef __VM_SYSTEM_H__
#define __VM_SYSTEM_H__

#include <memory>
#include <list>

#ifndef WIN32
#include <unistd.h>
#endif

#ifdef WIN32
#include <windows.h>
inline void mySleep(int x)
{
    Sleep(x);
}
#define VSLAM_SLEEP(x)  mySleep(x)
#else
#include <unistd.h>
#define VSLAM_SLEEP(x)  usleep(x*1000)
#endif //WIN32

#include <rvVM.h>
#include "VSLAMHijack.h"
#include "CameraInterface.h"

#ifdef OPENCV_ENABLED
#include "opencv2/opencv.hpp"
#endif

#ifdef ROS_BASED
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp> 
#include <nav_msgs/msg/odometry.hpp>
#elif ROS1_BASED
#include <ros/ros.h>
#endif

#ifdef SIMULATION
#include <condition_variable>
#endif //SIMULATION

typedef struct
{
    double w;
    double x;
    double y;
    double z;
}quaternion_type;

class VMSystem : public HijackReceiver
{
public:
    static std::shared_ptr<VMSystem> Initialize(const std::string& algSetting, std::shared_ptr<CameraInterface> camera, bool _showImg);

    static void Stop(int /*sig*/)
    {
        systemState = KSTOPPING;
#ifdef ROS_BASED
        rclcpp::shutdown();
#elif defined (ROS1_BASED)
        ros::shutdown();
#endif
    }

    static void Run();

    static void Spin();

    static void Quit(void);

    static void sleep();

    static void awake();

    static void restart();

    virtual ~VMSystem();
    static void deinit();

    VMSystem(std::shared_ptr<CameraInterface>& camera);
    static void addDepthImage(const int64_t timeStamp, const unsigned char* imageBuf, const unsigned short* depthBuf);
    static void state_callback(const std::string& msg);
    static void addCameraPose(const rvPose6DRTWithTimestamp& pose);

    static void setSystemWorking()
    {
        systemState = KWORKING;
    }

    static bool isSystemWorking()
    {
        return systemState == KWORKING;
    }

    static rvVM* vmPtr;

#ifdef ROS_BASED
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr cameraInMapPose_sub;
#endif
private:
    VMSystem(const VMSystem&) = delete;
    VMSystem& operator= (const VMSystem&) = delete;

    static std::shared_ptr<VMSystem> t;
    static std::shared_ptr<VSLAMHijack> hijack;
    static std::string algConfFile;
    static std::shared_ptr<CameraInterface> inputCamera;

    typedef enum
    {
        KSLEEPING = 0,    //No signal (image, wheel, IMU) input, coordinate invalid
        KWORKING,         //With signal input, coordinate built and valid
        KSTOPPING         //Try to kill all threads
    } SystemState;

    static SystemState systemState;
    void addHijack(bool status, int64_t timestamp);
    static rvCameraParams cameraConfiguration;

    static int width, height;


#ifdef SIMULATION
    //For simulation
    static std::mutex mut;
    static uint64_t currentImageTimeStamp;
    static uint64_t rawPoseTimeStamp;
    static std::condition_variable data_cond;
#endif //SIMULATION

#ifdef ROS_BASED
    void state_callbackROS(const std_msgs::msg::String::SharedPtr msg) const;
    void pose_callbackROS(const nav_msgs::msg::Odometry::SharedPtr msg) const;
#endif
};

#define VM_APP_VERSION "3.0.1.1"

#endif //__VM_SYSTEM_H__
