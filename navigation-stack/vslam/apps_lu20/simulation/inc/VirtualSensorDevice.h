/*******************************************************************************
@copyright
Copyright (c) 2017-2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#ifndef  __VIRTUAL_SENSOR_DEVICE_H__
#define __VIRTUAL_SENSOR_DEVICE_H__

#include "rv.h"
#include "rvCamera.h"
#include "SlamDataReader.h"

#include "CameraInterface.h"

#include <thread>
#include <mutex>


typedef void(*StateCallback)(const std::string& msg);
typedef void(*WheelCallback)(const sensor_wheel* sensorData);
typedef void(*PoseCallback)(const rvPose6DRTWithTimestamp&);
typedef  bool (*IsSystemWorkingFunc) ();
typedef  void (*WaitForRawPoseFunc) ();

class VirtualSensorDevice : public CameraInterface
{
    //DISALLOW_COPY_AND_ASSIGN(VirtualSensorDevice);
public:
    VirtualSensorDevice(const char* sensorSetting, const char* sensorType, int startFrame);

    ~VirtualSensorDevice();

    VirtualSensorDevice(const VirtualSensorDevice&) = delete;
    VirtualSensorDevice operator = (const VirtualSensorDevice&) = delete;

    bool start();

    bool stop();

    const rvCameraParams& getCameraConfiguration() const;
    const rvIMUConfiguration& getIMUConfiguration() const;
    const rvWheelConfiguration& getWheelConfiguration() const;
    const rvTargetImage & getTargetImage() const;

    void addCallback(CameraCallback callback);
    void addStateCallback(StateCallback callback);
    void addWheelCallback(WheelCallback callback);
    void addPoseCallback(PoseCallback callback);

    void setSensorSystemSyncFunc(IsSystemWorkingFunc systemWork, WaitForRawPoseFunc waitFunc)
    {
        isSystemWorking = systemWork;
        waitForRawPose = waitFunc;
    }

private:

    std::shared_ptr<std::thread> cameraThread;
    int sequence;
    void virtualSensorDeviceProc();

    bool ParsePlaybackParameters(const std::string& sensorType);

    std::string wheelodomName;
    std::string hijackName;

    std::string mTUMPath;

    std::mutex cameraMutex;
    std::vector<rvPose6DRTWithTimestamp> poseSet;

    int64_t timeOffset;  // in ns

    SlamDataReader* dataReader;
    mvFrame imageFrame;
    std::vector<sensor_wheel> wheelOdomSet;
    std::vector<imu_pack_dsp> imuSampleSet;
    std::vector<sensor_hijack> hijackSet;
    std::vector<StampedSystemCallback> callbackSet;

    int32_t formatVar;
    std::string Program_Root;

    bool stopNow;
    int startIndex;

    rvCameraParams cameraConfig;
    rvIMUConfiguration imuConfig;
    rvWheelConfiguration wheelConfig;
    rvTargetImage targetImage;

    CameraCallback cameraCallback;
    StateCallback stateCallback;
    WheelCallback wheelCallback;
    PoseCallback poseCallback;

    IsSystemWorkingFunc isSystemWorking;
    WaitForRawPoseFunc waitForRawPose;
};

#endif // ! __VIRTUAL_SENSOR_DEVICE_H__
