/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */
#include "sensor_service/comm/circule_buffer.hpp"

CycleBuffer::CycleBuffer(size_t item_size, int item_num, std::string sensor_name)
  : whole_num_(item_num), item_size_(item_size)
{
  name_ = "/sensor_service_" + sensor_name;
  current_index_ = 0;
  init();
}

void CycleBuffer::init()
{
  int shm_fd = shm_open(name_.c_str(), O_CREAT | O_RDWR, 0644);
  if (shm_fd == -1) {
    std::cout << "shm_open failed!" << std::endl;
    return;
  }
  if (ftruncate(shm_fd, item_size_ * whole_num_) == -1) {
    std::cout << "ftruncate failed!" << std::endl;
    close(shm_fd);
    return;
  }

  void * ptr = mmap(0, item_size_ * whole_num_, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
  if (ptr == MAP_FAILED) {
    std::cout << "mmap failed!" << std::endl;
    close(shm_fd);
    return;
  }

  fd_ = shm_fd;
  buffer_ptr_ = ptr;
  std::cout << "CycleBuffer init success!" << std::endl;
}

CycleBuffer::~CycleBuffer()
{
  if (fd_ != -1) {
    munmap(buffer_ptr_, item_size_ * whole_num_);
    shm_unlink(name_.c_str());
    close(fd_);
  }
}

sensors_event_t * CycleBuffer::get_writable_item()
{
  if (fd_ == -1) {
    std::cout << "CycleBuffer get_writable_item failed!!" << std::endl;
    return nullptr;
  }
  std::lock_guard<std::mutex> lock(mtx_);
  sensors_event_t * ptr = static_cast<sensors_event_t *>(buffer_ptr_) + current_index_;
  current_index_ = (current_index_ + 1) % whole_num_;
  return ptr;
}
