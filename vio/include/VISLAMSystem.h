/*****************************************************************************
Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries. 
All rights reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef __VISLAM_SYSTEM_H__
#define __VISLAM_SYSTEM_H__

#include <memory>

#include <unistd.h>
#define VSLAM_SLEEP(x)  usleep(x*1000)

#include <rvVIO.h>

#include "Visualization.h"
#include "CameraInterface.h"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <opencv2/opencv.hpp>

void Matrix2Quaternion(const float32_t a[3][4], float& qw, float& qx, float& qy, float& qz);

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

class VISLAMSystem
{
public:
   static std::shared_ptr<VISLAMSystem> Initialize(const std::string& algSetting, const std::string& outputDir);

   static void Stop(int /*sig*/)
    {
        systemState = KSTOPPING;
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
    static void addImageToVslam(const int64_t timeStamp, const uint8_t * imageBuf, const uint16_t * depthBuf);
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
	static void addIMU(const float linearAcceleration[3], const float angularVelocity[3], int64_t timestamp);
    static OutputRecorder recorder;

    static rvIMUConfiguration imuConfiguration;
    static rvWheelConfiguration wheelConfiguration;
    static bool loadWheelConfiguration(const char * configFile);

protected:
    static std::shared_ptr<VISLAMSystem> t;
    virtual void deinit0();
    
private:
    VISLAMSystem(const VISLAMSystem &) = delete;
    VISLAMSystem &operator= (const VISLAMSystem &) = delete;

    virtual void pub_camera_raw_pose(const rvVISLAMPose & pose) = 0;

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
    static rvCameraParams cameraConfiguration;
};

#define VSLAM_APP_VERSION "3.0.1.1"

#endif //__VISLAM_SYSTEM_H__
