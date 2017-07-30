/**
 * Copyright 2017 Massachusetts Institute of Technology
 *
 * @file types.h
 * @author W. Nicholas Greene
 * @date 2017-07-26 13:49:35 (Wed)
 */

#pragma once

#include <string>
#include <vector>

#include "dataset_utils/utils.h"

namespace dataset_utils {

namespace asl {

/**
 * @brief Raw IMU data read from ASL csv file
 */
struct IMUData {
  IMUData() : timestamp(0), gyro{0, 0, 0}, accel{0, 0, 0} {}

  /**
   * @brief Construct from ASL-formatted CSV string.
   *
   * Format: timestamp,gx,gy,gz,ax,ay,az
   */
  explicit IMUData(const std::string& csv) :
      timestamp(0), gyro{0, 0, 0}, accel{0, 0, 0} {
    parse(csv, ',', &timestamp, gyro, gyro + 1, gyro + 2,
          accel, accel + 1, accel + 2);
    return;
  }

  uint64_t timestamp; // Timestamp in nanoseconds.
  double gyro[3]; // Angular velocity [rad/sec].
  double accel[3]; // Linear acceleration [m/sec^2].
};

/**
 * @brief Pose data (e.g. Vicon or other 6DOF motion capture system).
 */
struct PoseData {
  PoseData() : timestamp(0), trans{0, 0, 0}, quat{0, 0, 0, 1} {}

  /**
   * @brief Construct from ASL-formatted CSV string.
   *
   * Format: timestamp,tx,ty,tz,qw,qx,qy,qz
   */
  explicit PoseData(const std::string& csv) :
      timestamp(0), trans{0, 0, 0}, quat{0, 0, 0, 1} {
    parse(csv, ',', &timestamp, trans, trans + 1, trans + 2,
          quat + 3, quat, quat + 1, quat + 2);
    return;
  }

  uint64_t timestamp; // Timestamp in nanoseconds.
  double trans[3]; // Translation.
  double quat[4]; // Orientation (xyzw order).
};

/**
 * @brief Position data (e.g. Leica).
 */
struct PositionData {
  PositionData() : timestamp(0), pos{0, 0, 0} {}

  /**
   * @brief Construct from ASL-formatted CSV string.
   *
   * Format: timestamp,tx,ty,tz
   */
  explicit PositionData(const std::string& csv) :
      timestamp(0), pos{0, 0, 0} {
    parse(csv, ',', &timestamp, pos, pos + 1, pos + 2);
    return;
  }

  uint64_t timestamp; // Timestamp in nanoseconds.
  double pos[3]; // Position data.
};

/**
 * @brief File data (e.g. for images or pointclouds stored in binary files).
 */
struct FileData {
  FileData() : timestamp(0), filename() {}

  /**
   * @brief Construct from ASL-formatted CSV string.
   *
   * Format: timestamp,filename
   */
  explicit FileData(const std::string& csv) :
      timestamp(0), filename() {
    parse(csv, ',', &timestamp, &filename);
    return;
  }

  uint64_t timestamp; // Timestamp in nanoseconds.
  std::string filename; // Filename.
};

}  // namespace asl

}  // namespace dataset_utils
