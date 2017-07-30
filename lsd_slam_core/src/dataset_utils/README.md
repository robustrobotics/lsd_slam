# dataset_utils
Utilities to read, write, and manipulate datasets in various formats.

## Maintainer
- W. Nicholas Greene (wng@csail.mit.edu)

## Dependencies
- gcc-5.4

## Installation
First, ensure that the dependencies are met and installed to a location where CMake can
find them. The rest of the build process follows the standard convention for CMake projects:
```bash
cd dataset_utils
mkdir build
cd build
cmake -D CMAKE_INSTALL_PREFIX=path/to/install/directory ..
make 
make install
```

## Usage
To include in a project, add the following to your `CMakeLists.txt`:
```cmake
find_package(dataset_utils)
include_directories(dataset_utils_INCLUDE_DIRS)
```
