/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */
#include "sensor_service/sensor/imu_sensor.hpp"

#include <iostream>

void IMUSensor::set_config(const int sample_rate)
{
  if (comm_ == nullptr) {
    std::cout << "please set communication at first!" << std::endl;
    return;
  }
  request_sample_rate_ = sample_rate;
  std::vector<int> available_frequency;
  comm_->get_sensor_available_sampling_frequency(SensorType::ACCELERATOR, available_frequency);
  comm_->get_sensor_available_sampling_frequency(SensorType::GYRO, available_frequency);

  std::cout << "Now imu sensor support sample_rate: " << std::endl;
  for (int i = 0; i < available_frequency.size(); i++) {
    std::cout << available_frequency[i] << " ";
  }
  std::cout << std::endl;

  for (auto support_frequency : available_frequency) {
    if (support_frequency >= request_sample_rate_) {
      adjust_sample_rate_ = support_frequency;
      break;
    }
  }
}

void IMUSensor::start_sensor()
{
  if (request_sample_rate_ == 0) {
    std::cout << "please set sensor config at first!" << std::endl;
    return;
  }

  comm_->start_sensor_sampling(SensorType::ACCELERATOR, request_sample_rate_, adjust_sample_rate_);
  comm_->start_sensor_sampling(SensorType::GYRO, request_sample_rate_, adjust_sample_rate_);
}

void IMUSensor::stop_sensor()
{
  comm_->stop_sensor_sampling(SensorType::ACCELERATOR);
  comm_->stop_sensor_sampling(SensorType::GYRO);
}

bool IMUSensor::get_fd(std::vector<int> & fds)
{
  int accel_fd, gyro_fd;
  bool accel_ret = comm_->get_sensor_fd(SensorType::ACCELERATOR, accel_fd);
  bool gyro_ret = comm_->get_sensor_fd(SensorType::GYRO, gyro_fd);
  if (!accel_ret || !gyro_ret) {
    return false;
  }
  fds.push_back(accel_fd);
  fds.push_back(gyro_fd);
  return true;
}

std::string IMUSensor::get_name()
{
  return "imu";
}
