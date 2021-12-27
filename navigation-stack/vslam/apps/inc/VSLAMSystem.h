/*****************************************************************************
@copyright
Copyright (c) 2020 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef __VSLAM_SYSTEM_H__
#define __VSLAM_SYSTEM_H__

#include <memory>

#ifndef WIN32
#include <unistd.h>
#endif

#include <rvVWSLAM.h>
#ifdef IMU_SUPPORTED
#include "VSLAMIMU.h"
#endif
#include "VSLAMWheel.h"
#include "VSLAMHijack.h"
#include "Visualization.h"


//#ifdef ARM_BASED
#ifdef ENABLE_DEPTH
#include "InputCamera_D435i.h"
#elif defined (ARM_BASED)
#include "InputCamera_ov9282.h"
//#endif
#elif defined (ENABLE_KINECT)
#include "InputCamera_Kinect2.h"
#else
#include "VirtualSensorDevice.h"
#endif
//#endif

#ifdef ROS_BASED
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <image_transport/image_transport.h>
#endif

#ifdef WIN32
#include <windows.h>
inline void mySleep( int x )
{
   Sleep( x );
}
#define VSLAM_SLEEP(x)  mySleep(x)
#else
#include <unistd.h>
#define VSLAM_SLEEP(x)  usleep(x*1000)
#endif //WIN32

#ifndef IMU_SUPPORTED
class VSLAMSystem: public WheelOdomReceiver, public HijackReceiver
#else
class VSLAMSystem: public WheelOdomReceiver, public IMUReceiver, public HijackReceiver
#endif
{
public:
   static std::shared_ptr<VSLAMSystem> Initialize( const std::string & root, const std::string & outputDir, bool _showImg );

   static void Stop(int sig)
    {
        systemState = KSTOPPING;
#ifdef ROS_BASED
	rclcpp::shutdown();
#endif
    }

    static void Run();

    static void Spin();

    static void Quit(void) {
       rvVWSLAM_Stop(vslamPtr);

#ifdef IMU_SUPPORTED
        if( imu )
           imu->stop();
#endif
        if( wheel )
           wheel->stop();
        //wod->stop();
        if( hijack )
           hijack->stop();

        //yk
        if(inputCamera)
            inputCamera->stop();
    }

    static void sleep(bool isCloseCamera = false);

    static void awake(bool isStartCamera = false);

    static void reset();

    virtual ~VSLAMSystem();
    static void deinit();

    VSLAMSystem();
    static bool showImg;
    static void addImageToVslam( const int64_t timeStamp, const uint8_t * imageBuf, const uint16_t * depthBuf );

    static void setSystemWorking()
    {
       systemState = KWORKING;
    }

    void getRobotPose(rvVSLAMPose & pose )
    {

       pose = rvVWSLAM_GetBaselinkPose(vslamPtr);
    }

    //float getWallAngle()
    //{
    //   return rvVWSLAM_GetWallAngle();
    //}

    //static std::shared_ptr<rvVWSLAM> vslamPtr;

    static rvVWSLAM* vslamPtr;
#ifdef ROS_BASED
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub;
#endif

private:
    //extern rvVWSLAM * vslamPtr;
    VSLAMSystem(const VSLAMSystem &) = delete;
    VSLAMSystem &operator= (const VSLAMSystem &) = delete;

    //std::shared_ptr<rvVWSLAM> vslamPtr;
    static std::shared_ptr<VSLAMSystem> t;
    //rvVWSLAM *vwSLAM = nullptr;
#ifdef IMU_SUPPORTED
    static std::shared_ptr<VSLAMIMU> imu;
#endif
    static std::shared_ptr<VSLAMWheel> wheel;
    static std::shared_ptr<VSLAMHijack> hijack;
    static std::shared_ptr<Visualiser> viz;
    static std::string rootPath;
    static std::string outputPath;

//#ifdef ARM_BASED
#ifdef ENABLE_DEPTH
    static std::shared_ptr<InputCamera_D435i> inputCamera;
#elif defined (ARM_BASED)
    static std::shared_ptr<InputCamera_OV9282> inputCamera;
//#endif
#elif defined (ENABLE_KINECT)
    static std::shared_ptr<InputCamera_Kinect2> inputCamera;
#else
    static std::shared_ptr<camera::VirtualSensorDevice> inputCamera;
#endif
//#endif
    
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
    static rvCameraParams configuration;

#ifdef ROS_BASED
    void state_callback(const std_msgs::msg::String::SharedPtr msg) const;
#endif
};

#define VSLAM_APP_VERSION "3.0.1.1"

#endif //__VSLAM_SYSTEM_H__
