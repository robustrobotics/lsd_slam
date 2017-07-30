/**
 * Copyright 2017 Massachusetts Institute of Technology
 *
 * @file dataset.h
 * @author W. Nicholas Greene
 * @date 2017-07-26 19:36:17 (Wed)
 */

#pragma once

#include <vector>
#include <string>
#include <algorithm>

#include <yaml-cpp/yaml.h>

#include "dataset_utils/utils.h"

namespace dataset_utils {

namespace asl {

/**
 * @brief Class that represents data in ASL format.
 *
 * An ASL dataset is a folder containing two files:
 *   1. A Yaml file containing metadata.
 *   2. A CSV file with one measurement per line.
 *
 * @tparam Data Type of data contained in this dataset.
 */
template <typename Data>
class Dataset final {
 public:
  Dataset() = default;

  /**
   * @brief Construct dataset from disk.
   *
   * @param[in] path Path to data folder.
   * @param[in] yaml Name of yaml file.
   * @param[in] csv Name of CSV file.
   */
  explicit Dataset(const std::string& path,
                   const std::string& yaml = "sensor.yaml",
                   const std::string& csv = "data.csv") :
      path_(),
      metadata_(),
      data_() {
    read(path, yaml, csv);
    return;
  }

  ~Dataset() = default;

  Dataset(const Dataset& rhs) = delete;
  Dataset& operator=(const Dataset& rhs) = delete;

  Dataset(Dataset&& rhs) = default;
  Dataset& operator=(Dataset&& rhs) = default;

  /**
   * @brief Read dataset from disk.
   *
   * @param[in] path Path to data folder.
   * @param[in] yaml Name of yaml file.
   * @param[in] csv Name of CSV file.
   */
  void read(const std::string& path,
            const std::string& yaml = "sensor.yaml",
            const std::string& csv = "data.csv") {
    path_ = path;

    // Strip trailing / in path if it exists.
    if (path_.back() == '/') {
      path_ = path_.substr(0, path_.size() - 1);
    }

    // Read in Yaml.
    metadata_ = YAML::LoadFile(path_ + "/" + yaml);

    // Read data into vector. Skip first line that describes columns.
    data_.clear();
    std::vector<std::string> lines = std::move(readLines(path_ + "/" + csv));
    for (int ii = 1; ii < lines.size(); ++ii) {
      data_.emplace_back(Data(lines[ii]));
    }
    return;
  }

  // Accessors.
  const std::string& path() const { return path_; }
  const YAML::Node& metadata() const { return metadata_; }
  const std::vector<Data>& data() const { return data_; }

  std::string& path() { return path_; }
  YAML::Node& metadata() { return metadata_; }
  std::vector<Data>& data() { return data_; }

  const Data& operator[](std::size_t idx) const { return data_[idx]; }
  Data& operator[](std::size_t idx) { return data_[idx]; }

  const std::size_t size() { return data_.size(); }

 private:
  std::string path_; // Path to containing folder.
  YAML::Node metadata_; // Metadata from Yaml file.
  std::vector<Data> data_; // Vector of parsed data.
};

}  // namespace asl

}  // namespace dataset_utils
