/* Copyright 2015 Massachusetts Institute of Technology */

#include <thread>

#include "stats_tracker/stats_tracker.h"

#include "gtest/gtest.h"

namespace {

TEST(StatsTrackerTest, ConstructorTest) {
  stats_tracker::StatsTracker stats;
  return;
}

TEST(StatsTrackerTest, SetTest) {
  stats_tracker::StatsTracker stats;

  stats.set("num_a");
  EXPECT_EQ(0, stats.stats("num_a"));

  return;
}

TEST(StatsTrackerTest, AddTest1) {
  stats_tracker::StatsTracker stats;

  stats.add("num_a", 9);
  EXPECT_EQ(9, stats.stats("num_a"));

  return;
}

TEST(StatsTrackerTest, AddTest2) {
  stats_tracker::StatsTracker stats;

  stats.add("num_a", 9);
  stats.add("num_a", 9);
  stats.add("num_a", 9);

  EXPECT_EQ(27, stats.stats("num_a"));

  return;
}

TEST(StatsTrackerTest, TickTockTest1) {
  stats_tracker::StatsTracker stats;

  float sleep_length = 1.0f;

  stats.tick("t1");
  sleep(sleep_length);
  float ret = stats.tock("t1");

  float tol = 10.0f;
  EXPECT_NEAR(sleep_length * 1000, ret, tol);

  return;
}

TEST(StatsTrackerTest, TickTockTest2) {
  stats_tracker::StatsTracker stats;

  float sleep_length = 1.0f;

  stats.tick("t1");
  sleep(sleep_length);
  stats.tock("t1");

  float tol = 10.0f;
  EXPECT_NEAR(sleep_length * 1000, stats.timings("t1"), tol);

  return;
}

TEST(StatsTrackerTest, SetPrefixTest) {
  stats_tracker::StatsTracker stats("a/");

  stats.set("num_a");
  EXPECT_EQ(0, stats.stats("num_a"));

  return;
}

TEST(StatsTrackerTest, AddPrefixTest1) {
  stats_tracker::StatsTracker stats("a/");

  stats.add("num_a", 9);

  EXPECT_EQ(9, stats.stats("num_a"));

  return;
}

TEST(StatsTrackerTest, AddPrefixTest2) {
  stats_tracker::StatsTracker stats("a/");

  stats.add("num_a", 9);
  stats.add("num_a", 9);
  stats.add("num_a", 9);

  EXPECT_EQ(27, stats.stats("num_a"));

  return;
}

TEST(StatsTrackerTest, TickTockPrefixTest1) {
  stats_tracker::StatsTracker stats("a/");

  float sleep_length = 1.0f;

  stats.tick("t1");
  sleep(sleep_length);
  float ret = stats.tock("t1");

  float tol = 10.0f;
  EXPECT_NEAR(sleep_length * 1000, ret, tol);

  return;
}

TEST(StatsTrackerTest, TickTockPrefixTest2) {
  stats_tracker::StatsTracker stats("a/");

  float sleep_length = 1.0f;

  stats.tick("t1");
  sleep(sleep_length);
  stats.tock("t1");

  float tol = 10.0f;
  EXPECT_NEAR(sleep_length * 1000, stats.timings("t1"), tol);

  return;
}

TEST(StatsTrackerTest, AddThreadTest1) {
  stats_tracker::StatsTracker stats;

  std::thread t1([&stats]() {
      stats.add("a", 1);
    });

  stats.add("a", 1);

  t1.join();

  EXPECT_EQ(2, stats.stats("a"));

  return;
}

TEST(StatsTrackerTest, AddThreadTest2) {
  stats_tracker::StatsTracker stats;

  std::thread t1([&stats]() {
      stats.add("a", 1);
    });

  stats.add("b", 1);

  t1.join();

  EXPECT_EQ(1, stats.stats("a"));
  EXPECT_EQ(1, stats.stats("b"));

  return;
}

TEST(StatsTrackerTest, TickTockThreadTest1) {
  stats_tracker::StatsTracker stats;

  float sleep_length = 1.0f;

  std::thread t1([&stats, sleep_length]() {
      stats.tick("a");
      sleep(sleep_length);
      stats.tock("a");
    });

  stats.tick("a");
  sleep(sleep_length);
  stats.tock("a");

  t1.join();

  float tol = 10.0f;
  EXPECT_NEAR(sleep_length * 1000, stats.timings("a"), tol);

  return;
}

TEST(StatsTrackerTest, TickTockThreadTest2) {
  stats_tracker::StatsTracker stats;

  float sleep_length = 1.0f;

  std::thread t1([&stats, sleep_length]() {
      stats.tick("a");
      sleep(sleep_length);
      stats.tock("a");
    });

  stats.tick("b");
  sleep(sleep_length);
  stats.tock("b");

  t1.join();

  float tol = 10.0f;
  EXPECT_NEAR(sleep_length * 1000, stats.timings("a"), tol);
  EXPECT_NEAR(sleep_length * 1000, stats.timings("b"), tol);

  return;
}

TEST(StatsTrackerTest, StatsCopyTest1) {
  stats_tracker::StatsTracker stats;

  stats.add("a", 10);

  auto sstats = stats.stats();

  stats.add("a", 10);

  EXPECT_EQ(10, sstats["a"]);
  EXPECT_EQ(20, stats.stats("a"));

  return;
}

TEST(StatsTrackerTest, StatsCopyTest2) {
  stats_tracker::StatsTracker stats;

  stats.add("a", 10);

  auto sstats = stats.stats();

  stats.add("b", 9);

  EXPECT_EQ(10, sstats["a"]);
  EXPECT_EQ(9, stats.stats("b"));
  EXPECT_EQ(1, sstats.size());

  return;
}

TEST(StatsTrackerTest, TimingsCopyTest1) {
  stats_tracker::StatsTracker stats;

  float sleep_length = 1.0f;

  stats.tick("a");
  sleep(sleep_length);
  stats.tock("a");

  auto timings = stats.timings();

  stats.tick("a");
  sleep(2*sleep_length);
  stats.tock("a");

  float tol = 10.0f;
  EXPECT_NEAR(sleep_length * 1000, timings["a"], tol);
  EXPECT_NEAR(2 * sleep_length * 1000, stats.timings("a"), tol);

  return;
}

}  // namespace
