#include "semantic_detection/utils/YamlParser.hpp"

#include <algorithm>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace semantic_detection {

std::vector<std::string> loadClassNames(const std::string & data_yaml_path) {
  YAML::Node root = YAML::LoadFile(data_yaml_path);
  YAML::Node names = root["names"];
  if (!names) {
    throw std::runtime_error("YamlParser: no 'names' key found in " + data_yaml_path);
  }

  std::vector<std::string> class_names;
  if (names.IsSequence()) {
    class_names.reserve(names.size());
    for (const auto & entry : names) {
      class_names.push_back(entry.as<std::string>());
    }
  } else if (names.IsMap()) {
    // Ultralytics data.yaml commonly uses "names: {0: person, 1: bicycle, ...}".
    std::vector<std::pair<int, std::string>> entries;
    for (const auto & kv : names) {
      entries.emplace_back(kv.first.as<int>(), kv.second.as<std::string>());
    }
    std::sort(entries.begin(), entries.end(),
              [](const auto & a, const auto & b) { return a.first < b.first; });
    class_names.reserve(entries.size());
    for (auto & [id, name] : entries) {
      class_names.push_back(std::move(name));
    }
  } else {
    throw std::runtime_error("YamlParser: 'names' in " + data_yaml_path +
                             " is neither a list nor a map");
  }
  return class_names;
}

}  // namespace semantic_detection
