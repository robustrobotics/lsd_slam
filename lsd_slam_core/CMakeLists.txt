# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(lsd_slam_core)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
set(ROS_SENSOR_STREAMS_REQ_PKGS
  roscpp
  image_transport
  message_filters
  tf2_ros
  sensor_msgs
  geometry_msgs
  cv_bridge
  camera_info_manager
  image_geometry)
find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure roslib rosbag lsd_slam_viewer ${ROS_SENSOR_STREAMS_REQ_PKGS})

# Required for ASL reader.
find_package(PkgConfig REQUIRED) # For projects with pkg-config files.
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp>=0.5)
include_directories(${YAML_CPP_INCLUDE_DIRS})

include_directories(include ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.8.7)
project(lsd_slam_core)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif ()

# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_init() 

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_MODULE_PATH   ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})

# Find my installation of OpenCV 2.4.8.
find_package(OpenCV REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(SuiteParse REQUIRED) # Apparently needed by g2o
find_package(G2O REQUIRED)

# FabMap
# uncomment this part to enable fabmap
#add_subdirectory(${PROJECT_SOURCE_DIR}/thirdparty/openFabMap)
#include_directories(${PROJECT_SOURCE_DIR}/thirdparty/openFabMap/include)
#add_definitions("-DHAVE_FABMAP")
#set(FABMAP_LIB openFABMAP )

# Dynamic Reconfigure Services
#rosbuild_find_ros_package(dynamic_reconfigure)
#include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
#gencfg()

# SSE flags

# CATKIN_MIGRATION
# Find other way to find SSE
# rosbuild_check_for_sse()
add_definitions("-DUSE_ROS")
add_definitions("-DENABLE_SSE")

# Also add some useful compiler flag
set(CMAKE_CXX_FLAGS
   "${CMAKE_CXX_FLAGS} ${SSE_FLAGS} -march=native  -std=c++0x"
) 

# Set source files
set(lsd_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/DataStructures/Frame.cpp
  ${PROJECT_SOURCE_DIR}/src/DataStructures/FramePoseStruct.cpp
  ${PROJECT_SOURCE_DIR}/src/DataStructures/FrameMemory.cpp
  ${PROJECT_SOURCE_DIR}/src/SlamSystem.cpp
  ${PROJECT_SOURCE_DIR}/src/LiveSLAMWrapper.cpp
  ${PROJECT_SOURCE_DIR}/src/DepthEstimation/DepthMap.cpp
  ${PROJECT_SOURCE_DIR}/src/DepthEstimation/DepthMapPixelHypothesis.cpp
  ${PROJECT_SOURCE_DIR}/src/util/globalFuncs.cpp
  ${PROJECT_SOURCE_DIR}/src/util/SophusUtil.cpp
  ${PROJECT_SOURCE_DIR}/src/util/settings.cpp
  ${PROJECT_SOURCE_DIR}/src/util/Undistorter.cpp
  ${PROJECT_SOURCE_DIR}/src/Tracking/Sim3Tracker.cpp
  ${PROJECT_SOURCE_DIR}/src/Tracking/Relocalizer.cpp
  ${PROJECT_SOURCE_DIR}/src/Tracking/SE3Tracker.cpp
  ${PROJECT_SOURCE_DIR}/src/Tracking/least_squares.cpp
  ${PROJECT_SOURCE_DIR}/src/Tracking/TrackingReference.cpp
  ${PROJECT_SOURCE_DIR}/src/IOWrapper/Timestamp.cpp
  ${PROJECT_SOURCE_DIR}/src/GlobalMapping/FabMap.cpp
  ${PROJECT_SOURCE_DIR}/src/GlobalMapping/KeyFrameGraph.cpp
  ${PROJECT_SOURCE_DIR}/src/GlobalMapping/g2oTypeSim3Sophus.cpp
  ${PROJECT_SOURCE_DIR}/src/GlobalMapping/TrackableKeyFrameSearch.cpp
)
set(SOURCE_FILES
  ${lsd_SOURCE_FILES}
  ${PROJECT_SOURCE_DIR}/src/IOWrapper/ROS/ROSImageStreamThread.cpp
  ${PROJECT_SOURCE_DIR}/src/IOWrapper/ROS/ROSOutput3DWrapper.cpp
  ${PROJECT_SOURCE_DIR}/src/IOWrapper/OpenCV/ImageDisplay_OpenCV.cpp
)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/thirdparty/Sophus
  ${CSPARSE_INCLUDE_DIR} #Has been set by SuiteParse
  ${CHOLMOD_INCLUDE_DIR} #Has been set by SuiteParse
  ${G2O_INCLUDE_DIR}
  ./src/stats_tracker/src # Needed for tracking CPU load.
)


# build shared library.
add_library(lsdslam SHARED ${SOURCE_FILES})
#target_link_libraries(lsdslam ${FABMAP_LIB} g2o_core g2o_stuff csparse cxsparse g2o_solver_csparse g2o_csparse_extension g2o_types_sim3 g2o_types_sba)
target_link_libraries(lsdslam ${FABMAP_LIB} 
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${G2O_TYPES_SBA}
  csparse cxsparse ${OpenCV_LIBS} ${catkin_LIBRARIES})
find_package(Boost REQUIRED COMPONENTS thread system)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(lsdslam ${Boost_LIBRARIES})

# build live ros node
add_executable(live_slam src/main_live_odometry.cpp)
target_link_libraries(live_slam lsdslam)


# build image node
add_executable(dataset src/main_on_images.cpp)
target_link_libraries(dataset lsdslam)

# build tum offline node.
include_directories(src/ros_sensor_streams/src)
add_executable(main_tum_offline src/main_tum_offline.cpp
  src/ros_sensor_streams/src/ros_sensor_streams/tum_rgbd_offline_stream.h
  src/ros_sensor_streams/src/ros_sensor_streams/tum_rgbd_offline_stream.cc)
target_link_libraries(main_tum_offline lsdslam)

# build asl offline node.
include_directories(src/ros_sensor_streams/src
  src/dataset_utils/src)
add_executable(main_asl_offline src/main_asl_offline.cpp
  src/ros_sensor_streams/src/ros_sensor_streams/asl_rgbd_offline_stream.h
  src/ros_sensor_streams/src/ros_sensor_streams/asl_rgbd_offline_stream.cc
  src/dataset_utils/src/dataset_utils/assert.h
  src/dataset_utils/src/dataset_utils/utils.h
  src/dataset_utils/src/dataset_utils/utils.cc
  src/dataset_utils/src/dataset_utils/asl/dataset.h
  src/dataset_utils/src/dataset_utils/asl/types.h)
target_link_libraries(main_asl_offline lsdslam
  ${YAML_CPP_LIBRARIES})

## Generate added messages and services with any dependencies listed here
#generate_messages(
#    #TODO DEPENDENCIES geometry_msgs std_msgs
#)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
generate_dynamic_reconfigure_options(cfg/LSDParams.cfg cfg/LSDDebugParams.cfg)
add_dependencies(live_slam ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(dataset ${${PROJECT_NAME}_EXPORTED_TARGETS})
catkin_package(
    DEPENDS cv_bridge dynamic_reconfigure sensor_msgs roslib rosbag lsd_slam_viewer
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)
