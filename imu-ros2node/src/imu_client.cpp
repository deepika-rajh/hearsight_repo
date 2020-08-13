/*
 * Copyright (c) 2020 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#include <iostream>
#include <unistd.h>
#include <sys/types.h>  // for open
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>   // for mmap
#include <string.h>
#include <sys/socket.h> // for socket

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
        std::cerr << "open imu map error" << std::endl;
        return false;
    }
    _map = (char *)mmap(0, MMAP_SIZE, PROT_READ, MAP_SHARED, _mmap_fd, 0);
    if (_map == NULL) {
        std::cerr << "mmap error" << std::endl;
        return false;
    }
    return true;
}

bool ImuClient::GetImuData(struct imu_pack_dsp *imu)
{
    memcpy(imu, _map, DATA_SIZE);
    return true;
}

bool ImuClient::ConnectServer()
{
    _socket_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (_socket_fd < 0) {
        std::cout << "error: create socket" << std::endl;
        return false;
    }

    struct sockaddr_un server_addr;
    server_addr.sun_family = AF_UNIX;
    strlcpy(server_addr.sun_path, SOCKET_PATH, strlen(SOCKET_PATH));

    int ret = connect(_socket_fd, (struct sockaddr *)&server_addr, sizeof(server_addr));
    if (ret < 0) {
        std::cout << "error: connect socket" << std::endl;
        return false;
    }
    return true;
}

bool ImuClient::SendMsg()
{
    int send_len = send(_socket_fd, &_msg, sizeof(_msg), 0);
    if (send_len < 0) {
        std::cout << "error: socket send" << std::endl;
        return false;
    }
    return true;
}

bool ImuClient::SendMsgStart()
{
    _msg.cmd = START;
    return SendMsg();
}

bool ImuClient::SendMsgStop()
{
    _msg.cmd = STOP;
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

int main()
{
    ImuClient client;
    struct imu_pack_dsp imuData;
    int count = 50;
    bool success = false;

    if (!client.InitMmap()) {
        std::cout << "error: init mmap" << std::endl;
        return 1;
    }

    if (!client.ConnectServer()) {
        std::cout << "error: connect server" << std::endl;
        return 1;
    }

    success = client.SendMsgStart();
    if (!success) {
        std::cout << "error: send msg START" << std::endl;
        return 1;
    }

    success = client.SendMsgConfigRate(200);
    if (!success) {
        std::cout << "error: send msg config rate" << std::endl;
        return 1;
    }

    while (count--)
    {
        client.GetImuData(&imuData);
        std::cout << imuData.acceloration_x << std::endl;
        usleep(200 * 1000);
    }

    return 0;
}
