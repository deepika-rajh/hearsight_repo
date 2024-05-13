/*****************************************************************************
@copyright
Copyright (c) 2020-2023 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef __VSLAM_SYSTEM_H__
#define __VSLAM_SYSTEM_H__

#include <memory>
#include <unistd.h>
#define VSLAM_SLEEP(x)  usleep(x*1000)

#include <rvVWSLAM.h>
#include "VSLAMIMU.h"
#include "VSLAMWheel.h"
#include "VSLAMHijack.h"
#include "Visualization.h"
#include "CameraInterface.h"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class VSLAMSystem: public WheelOdomReceiver, public IMUReceiver, public HijackReceiver
{
public:
   static std::shared_ptr<VSLAMSystem> Initialize( const std::string & algSetting, 
                                                   const std::string & outputDir, 
                                                   std::shared_ptr<CameraInterface> camera,
                                                   bool _showImg);

   static void Stop(int /*sig*/)
    {
        systemState = KSTOPPING;
        rclcpp::shutdown();
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

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub;
    static rvIMUConfiguration imuConfiguration;
    static rvWheelConfiguration wheelConfiguration;
    static rvTargetImage targetImage;
    static bool loadWheelConfiguration( const char * configFile );
private:
    VSLAMSystem(const VSLAMSystem &) = delete;
    VSLAMSystem &operator= (const VSLAMSystem &) = delete;

    static std::shared_ptr<VSLAMSystem> t;
    static std::shared_ptr<VSLAMIMU> imu;
    void addIMU( const float linearAcceleration[3], const float angularVelocity[3], int64_t timestamp );
    static std::shared_ptr<VSLAMWheel> wheel;

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
    void addHijack( bool status, int64_t timestamp );
    static rvCameraParams cameraConfiguration;
    void state_callbackROS(const std_msgs::msg::String::SharedPtr msg) const;
};

#define VSLAM_APP_VERSION "3.0.1.1"

#endif //__VSLAM_SYSTEM_H__
