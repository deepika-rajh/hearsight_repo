/*****************************************************************************
@copyright
Copyright (c) 2020-2023 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef __VSLAM_SYSTEM_H__
#define __VSLAM_SYSTEM_H__

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

#include <rvVWSLAM.h>
#ifdef IMU_SUPPORTED
#include "VSLAMIMU.h"
#endif
#include "VSLAMWheel.h"
#include "VSLAMHijack.h"
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


#ifndef IMU_SUPPORTED
class VSLAMSystem: public WheelOdomReceiver, public HijackReceiver
#else
class VSLAMSystem: public WheelOdomReceiver, public IMUReceiver, public HijackReceiver
#endif
{
public:
   static std::shared_ptr<VSLAMSystem> Initialize( const std::string & algSetting, 
                                                   const std::string & outputDir, 
                                                   std::shared_ptr<CameraInterface> camera,
                                                   bool _showImg);

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

    static void Quit( void );

    static void sleep(bool isCloseCamera = false);

    static void awake(bool isStartCamera = false);

    static void reset();

    virtual ~VSLAMSystem();
    static void deinit();

    VSLAMSystem( std::shared_ptr<CameraInterface> & camera );
    static void addImageToVslam( const int64_t timeStamp, const uint8_t * imageBuf, const uint16_t * depthBuf );
    static void state_callback(const std::string& msg);

    static bool isSystemWorking()
    {
       return systemState == KWORKING;
    }

    void getRobotPose(rvVSLAMPose & pose )
    {
       pose = rvVWSLAM_GetBaselinkPose(vslamPtr);
    }

    static void waitForRawPose();

    static rvVWSLAM* vslamPtr;
#ifdef ROS_BASED
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub;
#endif
    static rvIMUConfiguration imuConfiguration;
    static rvWheelConfiguration wheelConfiguration;
    static rvTargetImage targetImage;
    static bool loadWheelConfiguration( const char * configFile );
private:
    VSLAMSystem(const VSLAMSystem &) = delete;
    VSLAMSystem &operator= (const VSLAMSystem &) = delete;

    static std::shared_ptr<VSLAMSystem> t;
#ifdef IMU_SUPPORTED
    static std::shared_ptr<VSLAMIMU> imu;
#endif
    static std::shared_ptr<VSLAMWheel> wheel;
#ifdef HIJACK_SUPPORTED
    static std::shared_ptr<VSLAMHijack> hijack;
#endif //HIJACK_SUPPORTED
    static std::shared_ptr<Visualiser> viz;
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
    void addWheelOdom( float linearVelocity, float angualVelocity,
                  const float location[3], const float direction[4], int64_t timestamp );
    void addIMU( const float linearAcceleration[3], const float angularVelocity[3], int64_t timestamp );
    void addHijack( bool status, int64_t timestamp );
    static rvCameraParams cameraConfiguration;

	
#ifdef SIMULATION
    //For simulation
    static std::mutex mut;
    static uint64_t currentImageTimeStamp;
    static uint64_t rawPoseTimeStamp;
    static std::condition_variable data_cond;
#endif //SIMULATION

#ifdef ROS_BASED
    void state_callbackROS(const std_msgs::msg::String::SharedPtr msg) const;
#endif
};

#define VSLAM_APP_VERSION "3.0.1.1"

#endif //__VSLAM_SYSTEM_H__
