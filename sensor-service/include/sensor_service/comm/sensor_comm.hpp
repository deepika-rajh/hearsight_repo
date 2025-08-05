/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */
#ifndef SENSER_SERVICE__SENSOR_COMM_HPP_
#define SENSER_SERVICE__SENSOR_COMM_HPP_

#include <string>
#include <vector>

#define SENSOR_SIZE 2

enum class SensorType
{
  ACCELERATOR,
  GYRO,
  UNKNOWN
};

class SensorComm
{
public:
  /**
   *
   */
  virtual bool init() = 0;
  virtual bool start_sensor_sampling(const SensorType type,
      const int request_frequency,
      int & adjust_frequency) = 0;
  virtual void stop_sensor_sampling(const SensorType type) = 0;
  virtual bool get_sensor_available_sampling_frequency(const SensorType type,
      std::vector<int> & frequencies) = 0;
  virtual bool get_sensor_fd(const SensorType type, int & fd) = 0;

  virtual ~SensorComm(){};

protected:
  std::string convert_sensor_type_to_string(const SensorType sensor_type)
  {
    if (sensor_type == SensorType::ACCELERATOR) {
      return "accel";
    } else if (sensor_type == SensorType::GYRO) {
      return "gyro";
    }
    return "";
  }

  SensorType convert_string_to_sensor_type(const std::string sensor_type)
  {
    if (sensor_type == "accel") {
      return SensorType::ACCELERATOR;
    } else if (sensor_type == "gyro") {
      return SensorType::GYRO;
    }
    return SensorType::UNKNOWN;
  }
};

#endif