/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */
#ifndef SENSER_SERVICE__SESSION_COMM_HPP_
#define SENSER_SERVICE__SESSION_COMM_HPP_

#include <condition_variable>
#include <mutex>
#include <string>
#include <vector>

#include "ISession.h"
#include "SessionFactory.h"
#include "sensor_service/comm/circule_buffer.hpp"
#include "sensor_service/comm/sensor_comm.hpp"
#include "sns_cal.pb.h"
#include "sns_client.pb.h"
#include "sns_std.pb.h"
#include "sns_std_sensor.pb.h"
#include "sns_std_type.pb.h"
#include "sns_suid.pb.h"

class SessionComm : public SensorComm
{
public:
  bool init() override;
  bool start_sensor_sampling(const SensorType type,
      const int request_frequency,
      int & adjust_frequency) override;
  void stop_sensor_sampling(const SensorType type) override;
  bool get_sensor_available_sampling_frequency(const SensorType type,
      std::vector<int> & frequencies) override;
  bool get_sensor_fd(const SensorType type, int & fd) override;

private:
  int convert_sensor_type_to_index(const SensorType sensor_type)
  {
    if (sensor_type == SensorType::ACCELERATOR) {
      return 0;
    } else if (sensor_type == SensorType::GYRO) {
      return 1;
    }
    return -1;
  }

  SensorType convert_index_to_sensor_type(const int sensor_type)
  {
    if (sensor_type == 0) {
      return SensorType::ACCELERATOR;
    } else if (sensor_type == 1) {
      return SensorType::GYRO;
    }
    return SensorType::UNKNOWN;
  }

  bool get_suid(const SensorType type);
  bool get_attributes(const SensorType type);
  void event_process(const uint8_t * data,
      size_t size,
      uint64_t timeStamp,
      int index,
      bool process_data = false);

  bool init_{ false };
  std::shared_ptr<::com::quic::sensinghub::session::V1_0::sessionFactory> factory_;

  std::vector<std::shared_ptr<CycleBuffer>> buffer_list_;
  std::vector<std::vector<int>> sampling_frequencies_list_;
  std::vector<com::quic::sensinghub::suid> suid_list_;
  std::vector<com::quic::sensinghub::session::V1_0::ISession *> session_list_;

  std::vector<int> frequency_adapt_list_;

  std::mutex suid_mtx_;
  std::condition_variable suid_cv_;

  std::mutex attr_mtx_;
  std::condition_variable attr_cv_;

  std::mutex reserve_mtx_[SENSOR_SIZE];
  int reserve_count_[SENSOR_SIZE];
};

#endif