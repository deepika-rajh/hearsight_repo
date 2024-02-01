/*****************************************************************************
@copyright
Copyright (c) 2022-2023 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef __VISLAM_SYSTEM_H__
#define __VISLAM_SYSTEM_H__

#include <memory>

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

#include <rvVIO.h>

#ifdef IMU_SUPPORTED
#include "VSLAMIMU.h"
#endif
#include "Visualization.h"
#include "CameraInterface.h"

#ifdef ROS_BASED
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#elif ROS1_BASED
#include <ros/ros.h>
#endif

#ifdef SIMULATION
#include <condition_variable>
#endif //SIMULATION


class OutputRecorder
{
public:
   OutputRecorder();

   ~OutputRecorder();
   void initialize(const char* path);
   void write(int64_t timestamp, const rvVISLAMPose& pose);
   void deinit();

private:
   FILE* vioFp;
   FILE* vioFpTxt;
   FILE* fullStateFp;

};


#ifndef IMU_SUPPORTED
class VISLAMSystem: public WheelOdomReceiver, public HijackReceiver
#else
class VISLAMSystem: public IMUReceiver
#endif
{
public:
   static std::shared_ptr<VISLAMSystem> Initialize(const std::string& algSetting, const std::string& outputDir,
       std::shared_ptr<CameraInterface> camera, bool _showImg);

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

    static void sleep(bool isCloseCamera = false);

    static void awake(bool isStartCamera = false);

    static void reset();

    virtual ~VISLAMSystem();
    static void deinit();

    VISLAMSystem(std::shared_ptr<CameraInterface>& camera);
    static void addImageToVslam( const int64_t timeStamp, const uint8_t * imageBuf, const uint16_t * depthBuf );
    static void state_callback(const std::string& msg);

    static void setSystemWorking()
    {
       systemState = KWORKING;
    }

    static bool isSystemWorking()
    {
       return systemState == KWORKING;
    }

    static void waitForRawPose();

    static rvVIOHandle * vioPtr;
    static rvVISLAMMapPoint* pPoints;
    static int rvVIOPointsNum;

    static OutputRecorder recorder;

#ifdef ROS_BASED
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub;
#endif
    static rvIMUConfiguration imuConfiguration;
    static rvWheelConfiguration wheelConfiguration;
    static bool loadWheelConfiguration( const char * configFile );
private:
    VISLAMSystem(const VISLAMSystem &) = delete;
    VISLAMSystem &operator= (const VISLAMSystem &) = delete;

    static std::shared_ptr<VISLAMSystem> t;
#ifdef IMU_SUPPORTED
    static std::shared_ptr<VSLAMIMU> imu;
#endif

    static std::shared_ptr<Visualiser> viz;
    static std::string sensorPath;
    static std::string algConfFile;
    static std::string outputPath;
    static std::shared_ptr<CameraInterface> inputCamera;
        
    typedef enum
    {
       KSLEEPING = 0,    //No signal (image, wheel, IMU) input, coordinate invalid
       KWORKING,         //With signal input, coordinate built and valid
       KSTOPPING         //Try to kill all threads
    } SystemState;

    static SystemState systemState;
    void addIMU( const float linearAcceleration[3], const float angularVelocity[3], int64_t timestamp );
    static rvCameraParams cameraConfiguration;

	
#ifdef SIMULATION
    //For simulation
    static std::mutex mut;
    static int64_t currentImageTimeStamp;
    static int64_t rawPoseTimeStamp;
    static std::condition_variable data_cond;
#endif //SIMULATION

#ifdef ROS_BASED
    void state_callbackROS(const std_msgs::msg::String::SharedPtr msg) const;
#endif
};

#define VSLAM_APP_VERSION "3.0.1.1"

#endif //__VISLAM_SYSTEM_H__
