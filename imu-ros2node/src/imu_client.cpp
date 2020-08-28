/*
 * Copyright (c) 2020 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#include <iostream>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <string.h>
#include <sys/socket.h>
#include <bsd/string.h>

#include "imu-ros2node/imu_client.hpp"

ImuClient::~ImuClient()
{
    munmap(_map, MMAP_SIZE);
    close(_mmap_fd);
    close(_socket_fd);
}

bool ImuClient::InitMmap()
{
    _mmap_fd = open(MMAP_NAME, O_RDONLY);
    if (_mmap_fd < 0) {
        std::cerr << "imu client: open imu map failed" << std::endl;
        return false;
    }
    _map = (char *)mmap(0, MMAP_SIZE, PROT_READ, MAP_SHARED, _mmap_fd, 0);
    if (_map == NULL) {
        std::cerr << "imu client: mmap failed" << std::endl;
        return false;
    }
    std::cout << "imu client: mmap init succ" << std::endl;
    return true;
}

bool ImuClient::GetImuData(struct imu_pack_dsp *imu)
{
    int offset = PACK_GAP * 5;
    int data_size = sizeof(struct imu_pack_dsp);
    memcpy(imu, _map + MMAP_SIZE - offset, data_size);
    memcpy(imu, _map + (MMAP_SIZE / 2) - offset, data_size / 2);
    return true;
}

bool ImuClient::ConnectServer()
{
    _socket_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (_socket_fd < 0) {
        std::cout << "imu client: create socket failed" << std::endl;
        return false;
    }

    struct sockaddr_un server_addr;
    server_addr.sun_family = AF_UNIX;
    strlcpy(server_addr.sun_path, SOCKET_PATH, strlen(SOCKET_PATH) + 1);

    int ret = connect(_socket_fd, (struct sockaddr *)&server_addr, sizeof(server_addr));
    if (ret < 0) {
        std::cout << "imu client: connect socket " << SOCKET_PATH << " failed" << std::endl;
        return false;
    }
    return true;
}

bool ImuClient::SendMsg()
{
    int send_len = send(_socket_fd, &_msg, sizeof(_msg), 0);
    if (send_len < 0) {
        std::cout << "imu client: socket send failed" << std::endl;
        return false;
    }
    return true;
}

bool ImuClient::SendMsgStart(int sensor)
{
    _msg.cmd = START;
    _msg.data = sensor;
    return SendMsg();
}

bool ImuClient::SendMsgStop(int sensor)
{
    _msg.cmd = STOP;
    _msg.data = sensor;
    return SendMsg();
}

bool ImuClient::SendMsgConfigRate(int rate)
{
    _msg.cmd = CONFIG_RATE;
    _msg.data = rate;
    return SendMsg();
}

bool ImuClient::SendMsgConfigDataType(int type)
{
    _msg.cmd = CONFIG_DATATYPE;
    _msg.data = type;
    return SendMsg();
}
