/**
 * Copyright 2017 Massachusetts Institute of Technology
 *
 * @file dataset_test.cc
 * @author W. Nicholas Greene
 * @date 2017-07-26 23:42:00 (Wed)
 */

#include "gtest/gtest.h"

#include "dataset_utils/utils.h"

namespace dataset_utils {

/**
 * @brief Test associating two vectors.
 */
TEST(UtilsTest, AssociateTest1) {
  std::vector<int> a({1, 2, 3, 4});
  std::vector<float> b({0.0, 0.5, 1.01, 1.5, 2.01, 2.5, 3.01});

  std::vector<std::size_t> aidxs, bidxs;
  associate(a, b, &aidxs, &bidxs, [](int x, float y) { return (y - x) * (y - x); });

  EXPECT_EQ(3, aidxs.size());
  EXPECT_EQ(3, bidxs.size());

  EXPECT_EQ(0, aidxs[0]);
  EXPECT_EQ(2, bidxs[0]);

  EXPECT_EQ(1, aidxs[1]);
  EXPECT_EQ(4, bidxs[1]);

  EXPECT_EQ(2, aidxs[2]);
  EXPECT_EQ(6, bidxs[2]);

  return;
}

}  // namespace dataset_utils
