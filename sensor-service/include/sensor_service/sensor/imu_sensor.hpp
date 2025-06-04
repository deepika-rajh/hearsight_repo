/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */
#ifndef SENSER_SERVICE__IMU_SENSOR_HPP_
#define SENSER_SERVICE__IMU_SENSOR_HPP_

#include "sensor_service/sensor/sensor.hpp"

class IMUSensor : public Sensor
{
public:
  void start_sensor() override;
  void stop_sensor() override;
  void set_config(const int sample_rate) override;
  bool get_fd(std::vector<int> & fds) override;

  std::string get_name() override;
};

#endif