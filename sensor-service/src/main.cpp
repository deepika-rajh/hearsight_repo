/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#include <iostream>

#include "sensor_service/sensor_manager.hpp"

int main(int argc, char * argv[])
{
  SensorManager manager;
  bool ret = manager.init();
  if (!ret) {
    std::cout << "sensor service init failed!" << std::endl;
    return -1;
  }
  manager.run();
  return 0;
}