/*****************************************************************************
@copyright
Copyright (c) 2020 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/
#ifndef _EAGLE_IMU_H_
#define _EAGLE_IMU_H_

#include <thread> 
#include <mutex>
#include <memory>

#include <sensor-imu/sensor_imu_api.h>
#include <sensor-imu/sensor_datatypes.h>

class IMUReceiver
{
public:
   virtual void addIMU( const float linearAcceleration[3], const float angularVelocity[3], int64_t timestamp ) = 0;
};

class VSLAMIMU
{
public:
    VSLAMIMU(int32_t sign[3]);
    ~VSLAMIMU();
    
    bool init();

    int16_t getData( sensor_imu* dataArray, int32_t max_count, int32_t* available_imu_data )
    {
        int16_t result = -1;
        if( sensorHandlePtr != NULL)
        {
            result = sensor_imu_attitude_api_get_imu_raw( sensorHandlePtr, dataArray, max_count, available_imu_data );
        }
        return result;
    }

    const int32_t * getAxleSign()
    {
       return axleSign;
    }
    void stop();
    void start();

    void addReceiver( std::shared_ptr<IMUReceiver> &  receiver )
    {
       receiverVector.push_back( receiver );
    }

protected:
    VSLAMIMU(const VSLAMIMU &) = delete;
    VSLAMIMU &operator = (const VSLAMIMU &) = delete;

    void imuProc();
    bool threadRunning;
    sensor_handle* sensorHandlePtr;

    int32_t axleSign[3];

    std::shared_ptr<std::thread> imuPollThread;

    std::vector<std::shared_ptr<IMUReceiver>> receiverVector;

};

#endif
