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

#include <VWSLAM.h>
#include "VSLAMIMU.h"
#include "VSLAMWheel.h"
#include "VSLAMHijack.h"
#include "Visualization.h"


#ifdef ARM_BASED
#ifdef ENABLE_DEPTH
#include "InputCamera_D435i.h"
#else
#include "InputCamera_ov9282.h"
#endif
#else
#include "VirtualSensorDevice.h"
#endif

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


class VSLAMSystem: public WheelOdomReceiver, public IMUReceiver, public HijackReceiver
{
public:
   static std::shared_ptr<VSLAMSystem> Initialize( const std::string & root, const std::string & outputDir, bool _showImg);
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
       vwSLAM->stop();

        if( imu )
           imu->stop();
        if( wheel )
           wheel->stop();
        //wod->stop();
        if( hijack )
           hijack->stop();

        //yk
        if(inputCamera)
            inputCamera->stop();
    }

    static void sleep(bool isCloseCamera = true);

    static void awake();

    static void reset();

    virtual ~VSLAMSystem();
    VSLAMSystem();
    static bool showImg;
    static void addImageToVslam( const int64_t timeStamp, const uint8_t * imageBuf, const uint16_t * depthBuf );

    static void setSystemWorking()
    {
       systemState = KWORKING;
    }

    void getRobotPose( VWSLAM::VWSLAMPose & pose )
    {
       pose = vwSLAM->getVslamOutputPose();       
    }

    float getWallAngle()
    {
       return vwSLAM->getWallAngle();
    }

    static bool isMappingEnabled()
    {
       return doMapping;
    }


private:
    VSLAMSystem(const VSLAMSystem &) = delete;
    VSLAMSystem &operator= (const VSLAMSystem &) = delete;

    static std::shared_ptr<VSLAMSystem> t;
    static std::shared_ptr<VWSLAM> vwSLAM;
    static std::shared_ptr<VSLAMIMU> imu;
    static std::shared_ptr<VSLAMWheel> wheel;
    static std::shared_ptr<VSLAMHijack> hijack;
    static std::shared_ptr<Visualiser> viz;
    static std::string rootPath;
    static std::string outputPath;

#ifdef ARM_BASED
#ifdef ENABLE_DEPTH
    static std::shared_ptr<InputCamera_D435i> inputCamera;
#else
    static std::shared_ptr<InputCamera_OV9282> inputCamera;
#endif
#else
    static std::shared_ptr<camera::VirtualSensorDevice> inputCamera;
#endif
    
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

    static bool doMapping;
};

/* the first 3 numbers are CRD mv version */
#define VSLAM_APP_VERSION "1.2.7.1"

#endif //__VSLAM_SYSTEM_H__
