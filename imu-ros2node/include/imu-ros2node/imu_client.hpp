/*
 * Copyright (c) 2020 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#ifndef IMU_CLIENT
#define IMU_CLIENT

#include <stdint.h>
#include <sys/un.h>

#define DATA_SIZE 40
#define MMAP_SIZE DATA_SIZE
#define MMAP_NAME "/data/imu_map"
#define SOCKET_PATH "/data/imud_socket"

struct imu_pack_dsp {
	float acceloration_x;
	float acceloration_y;
	float acceloration_z;
	uint64_t time_acc;
	float angular_velocity_x;
	float angular_velocity_y;
	float angular_velocity_z;
	uint64_t time_gyro;
};

struct imud_ctl_msg {
    uint32_t cmd;
    int32_t  data;
};

#define START   0
#define STOP    1
#define CONFIG_RATE     2
#define CONFIG_DATATYPE 3
#define IMU_DATATYPE_NORMAL  0
#define IMU_DATATYPE_RAW     1

class ImuClient
{
public:
    ~ImuClient();
    bool InitMmap();
    bool GetImuData(struct imu_pack_dsp *imu);
    bool ConnectServer();
    bool SendMsgStart();
    bool SendMsgStop();
    bool SendMsgConfigRate(int rate);
    bool SendMsgConfigDataType(int type);
private:
    bool SendMsg();

    int _mmap_fd;
    char *_map = NULL;
    int _socket_fd;
    struct imud_ctl_msg _msg = {0, 0};
};

#endif
