/**
 * Copyright 2017 Massachusetts Institute of Technology
 *
 * @file dataset_test.cc
 * @author W. Nicholas Greene
 * @date 2017-07-26 23:42:00 (Wed)
 */

#include <cstdio>
#include <fstream>

#include <boost/filesystem.hpp>

#include "gtest/gtest.h"

#include "dataset_utils/utils.h"
#include "dataset_utils/asl/dataset.h"
#include "dataset_utils/asl/types.h"

namespace fs = boost::filesystem;

namespace dataset_utils {

namespace asl {

/**
 * @brief Test reading IMU sensor data.
 */
TEST(ASLDatasetTest, IMUDatasetTest) {
  // Get current working directory.
  char exe_str[200];
  readlink("/proc/self/exe", exe_str, 200);
  fs::path exe_path(exe_str);
  std::string base_dir = exe_path.parent_path().string();

  std::string path(base_dir + "/../data/V1_01/imu0");

  Dataset<IMUData> dataset(path);
  EXPECT_EQ(29120, dataset.size());

  EXPECT_EQ(1403715273262142976, dataset[0].timestamp);
  EXPECT_NEAR(0.0774926, dataset[0].gyro[2], 1e-6);

  EXPECT_EQ(1403715273307142912, dataset[9].timestamp);
  EXPECT_NEAR(0.079587, dataset[9].gyro[2], 1e-6);

  return;
}

/**
 * @brief Test reading pose sensor data.
 */
TEST(ASLDatasetTest, PoseDatasetTest) {
  // Get current working directory.
  char exe_str[200];
  readlink("/proc/self/exe", exe_str, 200);
  fs::path exe_path(exe_str);
  std::string base_dir = exe_path.parent_path().string();

  std::string path(base_dir + "/../data/V1_01/vicon0");

  Dataset<PoseData> dataset(path);
  EXPECT_EQ(14629, dataset.size());

  EXPECT_EQ(1403715271705179904, dataset[0].timestamp);
  EXPECT_NEAR(0.786802, dataset[0].trans[0], 1e-6);
  EXPECT_NEAR(0.993217, dataset[0].quat[3], 1e-6);

  EXPECT_EQ(1403715271795222016, dataset[9].timestamp);
  EXPECT_NEAR(0.786653, dataset[9].trans[0], 1e-6);
  EXPECT_NEAR(0.993184, dataset[9].quat[3], 1e-6);

  return;
}

/**
 * @brief Test reading position sensor data.
 */
TEST(ASLDatasetTest, PositionDatasetTest) {
  // Get current working directory.
  char exe_str[200];
  readlink("/proc/self/exe", exe_str, 200);
  fs::path exe_path(exe_str);
  std::string base_dir = exe_path.parent_path().string();

  std::string path(base_dir + "/../data/MH_02/leica0");

  Dataset<PositionData> dataset(path);
  EXPECT_EQ(2487, dataset.size());

  EXPECT_EQ(1403636856829881088, dataset[0].timestamp);
  EXPECT_NEAR(4.6870843035027319, dataset[0].pos[0], 1e-6);

  EXPECT_EQ(1403636857478881280, dataset[9].timestamp);
  EXPECT_NEAR(4.6870932500094726, dataset[9].pos[0], 1e-6);

  return;
}

/**
 * @brief Test reading file data.
 */
TEST(ASLDatasetTest, FileDatasetTest) {
  // Get current working directory.
  char exe_str[200];
  readlink("/proc/self/exe", exe_str, 200);
  fs::path exe_path(exe_str);
  std::string base_dir = exe_path.parent_path().string();

  std::string path(base_dir + "/../data/V1_01/cam0");

  Dataset<FileData> dataset(path);
  EXPECT_EQ(2912, dataset.size());

  EXPECT_EQ(1403715273262142976, dataset[0].timestamp);
  EXPECT_EQ("1403715273262142976.png", dataset[0].filename);

  EXPECT_EQ(1403715273712143104, dataset[9].timestamp);
  EXPECT_EQ("1403715273712143104.png", dataset[9].filename);

  return;
}

}  // namespace asl

}  // namespace dataset_utils
