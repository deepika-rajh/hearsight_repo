/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */
#ifndef SENSER_SERVICE__CIRCULE_BUFFER_HPP_
#define SENSER_SERVICE__CIRCULE_BUFFER_HPP_

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <mutex>

#include "sensor_service/comm/sns_direct_channel_buffer.h"

class CycleBuffer
{
public:
  CycleBuffer(){};
  CycleBuffer(size_t item_size, int item_num, std::string sensor_name);
  ~CycleBuffer();

  sensors_event_t * get_writable_item();

  int get_fd() { return fd_; };

private:
  void init();

  std::mutex mtx_;
  int current_index_;
  int whole_num_;
  size_t item_size_;
  int fd_{ -1 };
  void * buffer_ptr_{ nullptr };
  std::string name_;
};

#endif