/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */
#include "sensor_service/sensor_manager.hpp"

#include <stdio.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/un.h>

#include <thread>

bool SensorManager::create_sensor(const std::vector<SensorConfig> & sensor_config)
{
  for (int i = 0; i < sensor_config.size(); i++) {
    Sensor * sensor;
    if (sensor_config[i].name == "imu") {
      sensor = new IMUSensor();
    } else {
      std::cout << " unknown sensor type: " << sensor_config[i].name << std::endl;
      return false;
    }
    if (!sensor_config[i].use_iio) {
      sensor->set_sensor_comm(&comm_);
    }
    sensor->set_config(sensor_config[i].sample_rate);
    sensors_.push_back(sensor);
  }
  return true;
}

bool SensorManager::init()
{
  std::vector<SensorConfig> sensor_list;
  bool ret = parser_.read_config(sensor_list);
  if (!ret) {
    return ret;
  }
  for (int i = 0; i < SENSOR_SIZE; i++) {
    request[i] = 0;
  }
  comm_.init();
  ret = create_sensor(sensor_list);
  return ret;
}

void SensorManager::send_fd_to_client(const int socket_fd, std::vector<int> & fd_list)
{
  int fd_number = (int)fd_list.size();
  if (0 == fd_number) {
    std::cout << "there is no sensor fd!" << std::endl;
    return;
  }
  std::cout << "socket fd:" << socket_fd << std::endl;

  msghdr msg;
  cmsghdr * cm;
  int fds[fd_number];
  char buf[CMSG_SPACE(fd_number * sizeof(int))];

  char tmp[2];
  iovec iov[1];
  iov[0].iov_base = tmp;
  iov[0].iov_len = 2;
  msg.msg_iov = iov;
  msg.msg_iovlen = 1;
  msg.msg_name = nullptr;
  msg.msg_namelen = 0;

  msg.msg_control = buf;
  msg.msg_controllen = sizeof(buf);

  cm = CMSG_FIRSTHDR(&msg);
  cm->cmsg_level = SOL_SOCKET;
  cm->cmsg_type = SCM_RIGHTS;
  cm->cmsg_len = CMSG_LEN(fd_number * sizeof(int));

  // fd copy
  for (int i = 0; i < fd_number; i++) {
    fds[i] = fd_list[i];
    // set_fd_non_writable(fd_list[i]);
  }

  memcpy((int *)CMSG_DATA(cm), fds, fd_number * sizeof(int));
  int len = sendmsg(socket_fd, &msg, 0);
  if (len < 0) {
    perror("send msg failed");
  }
}

void SensorManager::get_sensor(int type, Sensor *& sensor)
{
  if (type == IMU) {
    for (int i = 0; i < sensors_.size(); i++) {
      if (sensors_[i]->get_name() == "imu") {
        sensor = sensors_[i];
        break;
      }
    }
  } else {
    sensor = nullptr;
  }
}

void SensorManager::close_connect(const int client_fd)
{
  mtx_.lock();
  if (client_num_ == 1) {
    for (int i = 0; i < SENSOR_SIZE; i++) {
      if (request[i] > 0) {
        Sensor * sensor;
        get_sensor(i, sensor);
        if (sensor != nullptr) {
          sensor->stop_sensor();
        }
        request[i] = 0;
      }
    }
  }
  client_num_--;
  mtx_.unlock();
  close(client_fd);
}

void SensorManager::run()
{
  umask(011);

  // create socket
  int service_fd = socket(AF_UNIX, SOCK_STREAM, 0);
  struct sockaddr_un server_addr;
  server_addr.sun_family = AF_UNIX;

  char server_socket_name[255] = SOCKET_PATH;
  unlink(server_socket_name);
  strlcpy(server_addr.sun_path, server_socket_name, sizeof(server_addr.sun_path));

  if (bind(service_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
    std::cout << "bind failed" << std::endl;
    exit(1);
  }

  if (listen(service_fd, MAX_SOCK_CLIENT) < 0) {
    std::cout << "Listen failed" << std::endl;
    exit(1);
  }

  struct sockaddr_un client_addr;
  socklen_t len = sizeof(client_addr);

  while (true) {
    int client_fd = accept(service_fd, (struct sockaddr *)&client_addr, &len);
    if (client_fd < 0) {
      std::cout << "accept failed" << std::endl;
      continue;
    }
    std::cout << "service accept from client:" << client_fd << std::endl;
    mtx_.lock();
    client_num_++;
    mtx_.unlock();
    auto func = [client_fd, this]() -> void {
      bool sensor_enable[SENSOR_SIZE];
      while (true) {
        int msg[2] = { -1, -1 };
        int len = recv(client_fd, &msg, sizeof(msg), 0);
        std::cout << "service recv from msg: " << msg[0] << "," << msg[1] << " len: " << len
                  << std::endl;
        if (len <= 0) {
          std::cerr << "sensor service ERROR: client exit unnormally, please check the client!"
                    << std::endl;
          close_connect(client_fd);
          break;
        }

        auto request_type = msg[0];
        auto sensor_type = msg[1];

        Sensor * sensor;
        get_sensor(sensor_type, sensor);
        if (sensor == nullptr && request_type != STOP) {
          std::cout << "unknow sensor type: " << msg[1] << std::endl;
          continue;
        }

        if (request_type == GETCONFIG) {
          int original_sample_rate, adjusted_sample_rate;
          original_sample_rate = sensor->get_request_sample_rate();
          adjusted_sample_rate = sensor->get_adjust_sample_rate();

          int msg_rate[2] = { original_sample_rate, adjusted_sample_rate };
          int send_len = send(client_fd, &msg_rate, sizeof(msg_rate), 0);
          if (send_len < 0) {
            std::cout << "sensor-service send sample_rate failed" << std::endl;
            close_connect(client_fd);
            break;
          }
        } else if (request_type == START) {
          mtx_.lock();
          if (request[sensor_type] == 0) {
            sensor->start_sensor();
          }
          request[sensor_type]++;
          sensor_enable[sensor_type] = true;
          mtx_.unlock();

          std::vector<int> fds;
          bool ret = sensor->get_fd(fds);
          send_fd_to_client(client_fd, fds);
        } else if (request_type == STOP) {
          mtx_.lock();
          if (sensor_type == SENSOR_SIZE) {
            for (int i = 0; i < SENSOR_SIZE; i++) {
              if (sensor_enable[i]) {
                request[i]--;
                if (request[i] == 0) {
                  Sensor * sensor;
                  get_sensor(i, sensor);
                  sensor->stop_sensor();
                }
              }
            }
            mtx_.unlock();
            close(client_fd);
            break;
          } else if (sensor_type > SENSOR_SIZE) {
            std::cout << "unknow sensor type: " << msg[1] << std::endl;
            mtx_.unlock();
            continue;
          }

          request[sensor_type]--;
          if (request[sensor_type] == 0) {
            Sensor * sensor;
            get_sensor(sensor_type, sensor);
            sensor->stop_sensor();
          }
          request[sensor_type]--;
          sensor_enable[sensor_type] = false;
          mtx_.unlock();
        } else {
          std::cout << "unknow request type: " << msg[0] << std::endl;
          continue;
        }
      }
    };
    auto thread = std::make_shared<std::thread>(func);
    thread->detach();
  }

  close(service_fd);
}