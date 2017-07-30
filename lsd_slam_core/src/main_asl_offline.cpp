/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam> 
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include "LiveSLAMWrapper.h"

#include <boost/thread.hpp>
#include "util/settings.h"
#include "util/globalFuncs.h"
#include "SlamSystem.h"

#include <sstream>
#include <fstream>
#include <dirent.h>
#include <algorithm>

#include <ros/ros.h>

#include "IOWrapper/ROS/ROSOutput3DWrapper.h"
#include "IOWrapper/ROS/rosReconfigure.h"

#include "ros_sensor_streams/asl_rgbd_offline_stream.h"

#include "util/Undistorter.h"
#include <ros/package.h>

#include "opencv2/opencv.hpp"

/**
 * @brief Find a parameter or fail.
 *
 * Copied from fsw/fla_utils/param_utils.h.
 */
template <typename T>
void getParamOrFail(const ros::NodeHandle& nh, const std::string& name, T* val) {
  if (!nh.getParam(name, *val)) {
    ROS_ERROR("Failed to find parameter: %s", nh.resolveName(name, true).c_str());
    exit(1);
  }
  return;
}

using namespace lsd_slam;
int main(int argc,  char** argv ) {
	ros::init(argc, argv, "LSD_SLAM");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  /*==================== Get input params  ====================*/
  std::string pose_path;
  getParamOrFail(pnh, "pose_path", &pose_path);

  std::string rgb_path;
  getParamOrFail(pnh, "rgb_path", &rgb_path);

  std::string depth_path;
  getParamOrFail(pnh, "depth_path", &depth_path);

  std::string world_frame_str;
  getParamOrFail(pnh, "world_frame", &world_frame_str);

  ros_sensor_streams::ASLRGBDOfflineStream::WorldFrame world_frame;
  if (world_frame_str == "RDF") {
    world_frame = ros_sensor_streams::ASLRGBDOfflineStream::WorldFrame::RDF;
  } else if (world_frame_str == "FLU") {
    world_frame = ros_sensor_streams::ASLRGBDOfflineStream::WorldFrame::FLU;
  } else if (world_frame_str == "FRD") {
    world_frame = ros_sensor_streams::ASLRGBDOfflineStream::WorldFrame::FRD;
  } else if (world_frame_str == "RFU") {
    world_frame = ros_sensor_streams::ASLRGBDOfflineStream::WorldFrame::RFU;
  } else {
    ROS_ERROR("Unknown world frame!\n");
    return 1;
  }

  std::string camera_frame_id;
  getParamOrFail(pnh, "camera_frame_id", &camera_frame_id);

  std::string camera_world_frame_id;
  getParamOrFail(pnh, "camera_world_frame_id", &camera_world_frame_id);

  double rate;
  getParamOrFail(pnh, "rate", &rate);

  /*==================== Set up dynamic reconfigure ====================*/
  dynamic_reconfigure::Server<lsd_slam_core::LSDParamsConfig> srv(pnh);
	srv.setCallback(dynConfCb);

	dynamic_reconfigure::Server<lsd_slam_core::LSDDebugParamsConfig> srvDebug(ros::NodeHandle("~Debug"));
	srvDebug.setCallback(dynConfCbDebug);

	packagePath = ros::package::getPath("lsd_slam_core")+"/";

  /*==================== Create input/output ====================*/
  auto input = std::make_shared<ros_sensor_streams::
                                ASLRGBDOfflineStream>(nh,
                                                      pose_path,
                                                      rgb_path,
                                                      depth_path,
                                                      "camera",
                                                      camera_world_frame_id,
                                                      camera_frame_id,
                                                      world_frame);

	// make output wrapper. just set to zero if no output is required.
  Output3DWrapper* outputWrapper = new ROSOutput3DWrapper(input->width(),
                                                          input->height());


	// make slam system
  SlamSystem* system = new SlamSystem(input->width(), input->height(), input->K(), doSlam);
	system->setVisualization(outputWrapper);

  /*==================== Main loop ====================*/
  int num_imgs = 0;
  ros::Rate ros_rate(rate);
  while (ros::ok() && !input->empty()) {
    // Grab data.
    uint32_t img_id;
    double time;
    cv::Mat3b rgb;
    cv::Mat1f depth;
    Eigen::Quaterniond q;
    Eigen::Vector3d t;
    input->get(&img_id, &time, &rgb, &depth, &q, &t);
    ROS_INFO("Got image %i at time %f", img_id, time);

    cv::Mat1b img_gray;
    cv::cvtColor(rgb, img_gray, cv::COLOR_RGB2GRAY);

    SE3 pose(q.cast<double>(), t.cast<double>());

    // Process image.
    if (num_imgs == 0) {
      system->randomInit(img_gray.data, time, img_id, pose);
    } else {
      system->trackFrame(img_gray.data, img_id , false, time, &pose);
    }

    ros::spinOnce();
    ros_rate.sleep();
    num_imgs++;
  }

  // system->finalize();

  ROS_INFO("Finished processing.");

	delete system;
  delete outputWrapper;
	return 0;
}
