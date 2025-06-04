/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */
#ifndef SENSER_SERVICE__SENSOR_HPP_
#define SENSER_SERVICE__SENSOR_HPP_

#include <string>

#include "sensor_service/comm/sensor_comm.hpp"

class Sensor
{
public:
  virtual void start_sensor() = 0;
  virtual void stop_sensor() = 0;
  virtual void set_config(const int sample_rate) = 0;
  virtual bool get_fd(std::vector<int> & fds) = 0;

  void set_sensor_comm(SensorComm * comm) { comm_ = comm; }

  int get_adjust_sample_rate() { return adjust_sample_rate_; }

  int get_request_sample_rate() { return request_sample_rate_; }

  virtual std::string get_name() { return ""; }

protected:
  SensorComm * comm_{ nullptr };
  int request_sample_rate_;
  int adjust_sample_rate_;
};

#endif