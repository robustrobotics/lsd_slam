# stats_tracker
A simple utility to keep track of algorithm statistics, counts, and load.

## Maintainer
- W. Nicholas Greene (wng@csail.mit.edu)

## Dependencies
- gcc-4.8.4

## Installation
First, ensure that the dependencies are met and installed to a location where CMake can
find them. The rest of the build process follows the standard convention for CMake projects:
```bash
cd stats_tracker
mkdir build
cd build
cmake -D CMAKE_INSTALL_PREFIX=path/to/install/directory ..
make 
make install
```

## Usage
To include in a project, add the following to your `CMakeLists.txt`:
```cmake
find_package(stats_tracker)
include_directories(stats_tracker_INCLUDE_DIRS)
```

Usage is like so:
```c++
#include <stats_tracker/stats_tracker.h>
#include <stats_tracker/load_tracker.h>

stats_tracker::StatsTracker stats("alg/");

// Time a function.
stats.tick("function");
function();
stats.tock("function");

std::cout << "alg/function took " << stats.timings("function") << " ms" << std::endl;

// Keep a running count.
stats.add("num_something", 10);
stats.add("num_something", 9);
stats.add("num_something", 8);

std::cout << "alg/num_something = " << stats.counts("num_something") << std::endl;

// Return copies of the internal maps.
// Don't return internal references because that would require the mutex to be locked.
auto counts = stats.counts();
auto timings = stats.timings();

// Track load for a process.
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
```
