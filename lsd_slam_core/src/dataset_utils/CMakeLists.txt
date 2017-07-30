cmake_minimum_required(VERSION 2.8)

# Configure CCache if available
find_program(CCACHE_FOUND ccache)
if (CCACHE_FOUND)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif (CCACHE_FOUND)

#-------------------------------------------------------------------------------
# Introduction.
#-------------------------------------------------------------------------------

# This is the top-level CMakeLists.txt file for dataset_utils.
# It will:
#   1. Build, install, and configure a library (dataset_utils).
#   2. Build and install an executble (dataset_utils_main).
#   3. Build unit tests (dataset_utils_test).
# 
# The build process follows the standard CMake convention:
#    cd dataset_utils
#    mkdir build
#    cd build
#    cmake -D CMAKE_INSTALL_PREFIX=path/to/install/directory ..
#    make 
#    make install
# Note that the default install directory is dataset_utils/install

#-------------------------------------------------------------------------------
# Project initialization.
#-------------------------------------------------------------------------------

# Here we tell cmake that we want to make a project called "dataset_utils", which will
# automatically create a variable called PROJECT_NAME (set to "dataset_utils"). We then
# tell CMake the version number of the project.
project(dataset_utils)
set(${PROJECT_NAME}_VERSION 0.0.1)

# Options
# Specify any custom command-line flags that you want to pass to CMake here.
# For example, if the user calls cmake with "cmake -DBUILD_TESTS", then the 
# BUILD_TESTS variable will be set to TRUE/ON and appropriate action will take
# place.
option(BUILD_DOCUMENTATION "Use Doxygen to create the HTML based API documentation" OFF)
option(BUILD_TESTS "Use GTest to build and test libraries" ON)

option(WITH_COVERAGE "Compile with code coverage" OFF)
if (WITH_COVERAGE)
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0 -fprofile-arcs -ftest-coverage")
  set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g -O0 -fprofile-arcs -ftest-coverage")
  set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -fprofile-arcs -ftest-coverage")
endif ()

add_definitions("-std=c++11")

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif (NOT CMAKE_BUILD_TYPE)

# CMake includes
# dataset_utils is packaged with some helper CMake functions. We include
# them here so that we can use them.
include(cmake/setup.cmake)
include(cmake/pkg_config.cmake)

# Locate external dependencies
# dataset_utils requires some external projects. We locate them here using
# find_package(). If the external project was configured using pkg-config
# instead of CMake, use the FindPkgConfig module instead.
# pkg_check_modules(EIGEN3 REQUIRED eigen3>=3.2)
# find_package(OpenCV REQUIRED core highgui imgproc)

find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc imgcodecs)
find_package(Boost 1.54 REQUIRED COMPONENTS system filesystem program_options)
find_package(PCL 1.7 REQUIRED COMPONENTS common io)
add_definitions(${PCL_DEFINITIONS})

find_package(PkgConfig REQUIRED) # For projects with pkg-config files.
pkg_check_modules(EIGEN3 REQUIRED eigen3>=3.2)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp>=0.5)

# Project includes
# We set the include directories for the project that are needed to compile
# the library and the executable.
# include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
include_directories(SYSTEM src
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS})

#-------------------------------------------------------------------------------
# Build, install, and configure a library.
#-------------------------------------------------------------------------------

# Build the library
# Here we specify the commands needed to compile the library.
# We put the library name into a variable for clarity. We then
# declare the source files needed for the library, and then call
# the add_library() command with the SHARED option to create a 
# shared object (dynamically-linked library). We then link the library
# we just created with the external libraries specified by our external
# projects using target_link_libraries(). Finally, we set some target
# properties.
# set(MY_LIBRARY_NAME dataset_utils)
# set(MY_LIBRARY_SRCS src/dataset_utils/dataset_utils.cc)
# add_library(${MY_LIBRARY_NAME} SHARED ${MY_LIBRARY_SRCS})
# # target_link_libraries(${MY_LIBRARY_NAME} ${OpenCV_LIBS})
# set_target_properties(${MY_LIBRARY_NAME} PROPERTIES 
#     COMPILE_FLAGS "-std=c++11 -Wall")

# Install the library and header files
# We've built the library, but we heven't put it anywhere useful.
# The install command copies the library to the directory set by
# CMAKE_INSTALL_PREFIX (by default it is set to the source folder).
# We install the header files to CMAKE_INSTALL_PREFIX/include.
# install(TARGETS ${MY_LIBRARY_NAME} 
#   RUNTIME DESTINATION bin
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib)
# install(DIRECTORY src/${PROJECT_NAME} DESTINATION include
#   FILES_MATCHING PATTERN "*.h")

add_subdirectory(src)

# Configure the library
# We've built the library and installed it, but we haven't told anyone else
# where to find it. The configuration process allows other users to find out
# where the library was installed and which headers/libraries to include 
# in order to use it.

# Configuration method 1: pkg_config
install_pkg_config_file(${PROJECT_NAME}
  DESCRIPTION "Utilities to read, write, and manipulate datasets in various formats."
  LIBS 
  VERSION ${${PROJECT_NAME}_VERSION}
  REQUIRES
  CFLAGS "-std=c++11")

# Configuration method 2: pure cmake, no import/export
# NOTE: You need to edit the config file (${PROJECT_NAME}Config.cmake.in) manually!
configure_file(${PROJECT_SOURCE_DIR}/cmake/templates/${PROJECT_NAME}Config.cmake.in
  "${PROJECT_BINARY_DIR}/lib/cmake/${PROJECT_NAME}/${PROJECT_NAME}Config.cmake" @ONLY)
configure_file(${PROJECT_SOURCE_DIR}/cmake/templates/${PROJECT_NAME}ConfigVersion.cmake.in
  "${PROJECT_BINARY_DIR}/lib/cmake/${PROJECT_NAME}/${PROJECT_NAME}ConfigVersion.cmake" @ONLY)
install(FILES 
  ${PROJECT_BINARY_DIR}/lib/cmake/${PROJECT_NAME}/${PROJECT_NAME}Config.cmake 
  ${PROJECT_BINARY_DIR}/lib/cmake/${PROJECT_NAME}/${PROJECT_NAME}ConfigVersion.cmake 
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/cmake/${PROJECT_NAME})

#-------------------------------------------------------------------------------
# Build and install an executable.
#-------------------------------------------------------------------------------

# Now we're ready to build an exexutable that uses the library we just
# created. First we specify the executable name and source files in variables
# for clarity. We then create the executable with the add_executable() command.
# Next, we link the executable to both the generated library as well as the
# external libraries. Finally, we set some target properties.
# set(MY_EXECUTABLE_NAME dataset_utils_main)
# set(MY_EXECUTABLE_SRC src/dataset_utils_main.cc)
# add_executable(${MY_EXECUTABLE_NAME} ${MY_EXECUTABLE_SRC})
# target_link_libraries(${MY_EXECUTABLE_NAME} ${MY_LIBRARY_NAME})
# set_target_properties(${MY_EXECUTABLE_NAME} PROPERTIES 
#     COMPILE_FLAGS "-std=c++11 -Wall")

# Install the executable.
# We've built an executable, but we haven't placed it anywhere useful.
# The install() command will place the binary file into CMAKE_INSTALL_PREFIX/bin.
# install(TARGETS ${MY_EXECUTABLE_NAME}
#   RUNTIME DESTINATION bin
#   LIBRARY DESTINATION lib 
#   ARCHIVE DESTINATION lib)

#-------------------------------------------------------------------------------
# Build the unit tests.
#-------------------------------------------------------------------------------

# If the BUILD_TESTS option is set, a Google test binary will be compiled.
if (BUILD_TESTS)
  add_subdirectory(./test)
endif()

#-------------------------------------------------------------------------------
# Build the doxygen documentation.
#-------------------------------------------------------------------------------
if (BUILD_DOCUMENTATION)
    find_package(Doxygen)

    if (NOT DOXYGEN_FOUND)
        message(FATAL_ERROR
        "Doxygen is needed to build the documentation.")
    endif()

    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/doc/Doxyfile.in
       ${CMAKE_CURRENT_BINARY_DIR}/doc/Doxyfile @ONLY)
    add_custom_target(doc ALL
       ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/doc/Doxyfile
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/doc
    COMMENT "Generatin API documentation with Doxygen" VERBATIM
    )
    install(DIRECTORY ${CMAKE_BINARY_DIR}/doc/html DESTINATION doc/${PROJECT_NAME})
    install(DIRECTORY ${CMAKE_BINARY_DIR}/doc/latex DESTINATION doc/${PROJECT_NAME})
endif()

