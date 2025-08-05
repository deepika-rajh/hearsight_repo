/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */
#ifndef SENSER_SERVICE__CONFIG_PARSER_HPP_
#define SENSER_SERVICE__CONFIG_PARSER_HPP_

#include <string>
#include <vector>

#define CONFIG_FILE_PATH ("/etc/sensors_info.conf")
#define CONFIG_FILE_DEFAULT_PATH ("/etc/sensors_info_conf.default")

struct SensorConfig
{
  std::string name;
  int sample_rate;
  bool use_iio;
};

class ConfigParser
{
public:
  bool read_config(std::vector<SensorConfig> & sensor);

private:
  bool load_config(std::string path);
  std::vector<SensorConfig> sensors;
};

#endif