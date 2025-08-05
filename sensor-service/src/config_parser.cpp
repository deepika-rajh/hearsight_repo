/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */
#include "sensor_service/config_parser.hpp"

#include <fstream>
#include <iostream>
#include <string>

#include "yaml-cpp/yaml.h"

bool ConfigParser::load_config(std::string path)
{
  try {
    YAML::Node config = YAML::LoadFile(path);
    for (auto it = config["sensor_list"].begin(); it != config["sensor_list"].end(); ++it) {
      SensorConfig sensor;
      sensor.name = it->as<std::string>();

      if (!config[sensor.name] || config[sensor.name].IsNull()) {
        return false;
      }

      YAML::Node node = config[sensor.name];
      sensor.sample_rate = node["sample_rate"].as<int>();

      if (!node["use_iio"] || node["use_iio"].IsNull()) {
        sensor.use_iio = false;
        sensors.emplace_back(sensor);
        continue;
      }

      sensor.use_iio = node["use_iio"].as<bool>();
      std::cout << "sensor name: " << sensor.name << " sample rate: " << sensor.sample_rate
                << std::endl;
      sensors.emplace_back(sensor);
    }
    return true;
  } catch (const YAML::ParserException & e) {
    std::cerr << "YAML: " << path << " parsing error: " << e.what() << std::endl;
  } catch (const YAML::BadFile & e) {
    std::cerr << "Error no such file: " << e.what() << std::endl;
  }
  return false;
}

bool ConfigParser::read_config(std::vector<SensorConfig> & sensor)
{
  if (!sensors.empty()) {
    sensor = sensors;
    return true;
  }

  bool ret = load_config(CONFIG_FILE_PATH);
  if (!ret) {
    std::cout << "There is no sensors_info conf or parser conf failed, use default conf."
              << std::endl;
    ret = load_config(CONFIG_FILE_DEFAULT_PATH);
    if (!ret) {
      std::cout << "No default conf file found or parser conf failed!" << std::endl;
      return false;
    }
  }

  sensor = sensors;
  return true;
}