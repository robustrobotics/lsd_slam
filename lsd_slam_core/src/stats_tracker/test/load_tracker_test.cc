/**
 * Copyright 2017 Massachusetts Institute of Technology
 *
 * @file load_tracker_test.cc
 * @author W. Nicholas Greene
 * @date 2017-02-04 19:56:23 (Sat)
 */

#include "stats_tracker/load_tracker.h"

#include "gtest/gtest.h"

namespace stats_tracker {

/**
 * @brief Query load periodically while sleeping.
 *
 * Not sure how to effectively test. What I did during development is pick a PID
 * and compare output with htop.
 */
TEST(LoadTrackerTest, IdleTest) {
  LoadTracker load_tracker(getpid());

  int dt_ms = 500;
  int num_iters = 1000;
  for (int ii = 0; ii < num_iters; ++ii) {
    Load max_load, sys_load, pid_load;

    load_tracker.get(&max_load, &sys_load, &pid_load);

    printf("max_load.cpu = %f\n", max_load.cpu);
    printf("max_load.mem = %lu\n", max_load.mem);
    printf("max_load.swap = %lu\n", max_load.swap);

    printf("sys_load.cpu = %f\n", sys_load.cpu);
    printf("sys_load.mem = %lu\n", sys_load.mem);
    printf("sys_load.swap = %lu\n", sys_load.swap);

    printf("pid_load.cpu = %f\n", pid_load.cpu);
    printf("pid_load.mem = %lu\n", pid_load.mem);
    printf("pid_load.swap = %lu\n", pid_load.swap);

    std::this_thread::sleep_for(std::chrono::milliseconds(dt_ms));
  }

  return;
}

}  // namespace stats_tracker
