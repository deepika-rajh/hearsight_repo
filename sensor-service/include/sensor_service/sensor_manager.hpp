/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */
#ifndef SENSER_SERVICE__SENSOR_MANAGER_HPP_
#define SENSER_SERVICE__SENSOR_MANAGER_HPP_

#include <mutex>
#include <vector>

#include "sensor_service/comm/session_comm.hpp"
#include "sensor_service/config_parser.hpp"
#include "sensor_service/sensor/imu_sensor.hpp"

#define SOCKET_PATH "/dev/shm/server_socket"
#define MAX_SOCK_CLIENT 10

#define GETCONFIG 0
#define START 1
#define STOP 2

#define IMU 0
#define SENSOR_SIZE 1

class SensorManager
{
public:
  bool init();
  void run();

private:
  void close_connect(const int client_fd);
  bool create_sensor(const std::vector<SensorConfig> & sensor_config);
  void send_fd_to_client(const int socket_fd, std::vector<int> & fd_list);
  void get_sensor(int type, Sensor *& sensor);

  int request[SENSOR_SIZE];
  int client_num_{ 0 };
  std::mutex mtx_;
  ConfigParser parser_;
  SessionComm comm_;
  std::vector<Sensor *> sensors_;
};

#endif