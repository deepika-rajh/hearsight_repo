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
#include <vector>

#include "imu_client.hpp"

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

    int32_t axleSign[3];

    //time stamp
    int64_t realClock;
    int64_t monotonicClock;
    int64_t clockOffset;

    std::shared_ptr<std::thread> imuPollThread;

    std::vector<std::shared_ptr<IMUReceiver>> receiverVector;

};

#endif
