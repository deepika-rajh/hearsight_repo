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

#include <unistd.h>
#define VSLAM_SLEEP(x)  usleep(x*1000)

#include <rvVM.h>
#include "VSLAMHijack.h"
#include "CameraInterface.h"

#include "opencv2/opencv.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp> 
#include <nav_msgs/msg/odometry.hpp>

typedef struct
{
    double w;
    double x;
    double y;
    double z;
}quaternion_type;


void Quaternion2Matrix( quaternion_type *q, double *m );
void stopExternalElements();
void showOccupancyImg(const cv::Mat & gridImage);
void showDepthImage(const cv::Mat& colorsMap);

class VMSystem : public HijackReceiver
{
public:
    static std::shared_ptr<VMSystem> Initialize(const std::string& algSetting, std::shared_ptr<CameraInterface> camera);

    static void Stop(int)
    {
        systemState = KSTOPPING;
        stopExternalElements();

        rclcpp::shutdown();
    }

    virtual void Run();

    virtual void Spin();

    virtual void Quit(void);

    static void sleep();

    static void awake();

    static void restart();

    virtual ~VMSystem();
    virtual void deinit();

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

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr cameraInMapPose_sub;
protected:

    typedef enum
    {
        KSLEEPING = 0,    //No signal (image, wheel, IMU) input, coordinate invalid
        KWORKING,         //With signal input, coordinate built and valid
        KSTOPPING         //Try to kill all threads
    } SystemState;

    static SystemState systemState;

    static int width, height;
    static rvCameraParams cameraConfiguration;

    static std::string algConfFile;

    static std::shared_ptr<VMSystem> t;

private:
    VMSystem(const VMSystem&) = delete;
    VMSystem& operator= (const VMSystem&) = delete;

    static std::shared_ptr<VSLAMHijack> hijack;
    static std::shared_ptr<CameraInterface> inputCamera;

    void addHijack(bool status, int64_t timestamp);

    static uint16_t * depthImage;

    void state_callbackROS(const std_msgs::msg::String::SharedPtr msg) const;
    void pose_callbackROS(const nav_msgs::msg::Odometry::SharedPtr msg) const;
};

#define VM_APP_VERSION "3.0.1.1"

#endif //__VM_SYSTEM_H__
