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

#include "ros_sensor_streams/tum_rgbd_offline_stream.h"

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
  std::string input_file;
  getParamOrFail(pnh, "input_file", &input_file);

  std::string calib_file;
  getParamOrFail(pnh, "calib_file", &calib_file);

  std::string input_frame_str;
  getParamOrFail(pnh, "input_frame", &input_frame_str);

  ros_sensor_streams::TUMRGBDOfflineStream::InputFrame input_frame;
  if (input_frame_str == "RDF") {
    input_frame = ros_sensor_streams::TUMRGBDOfflineStream::InputFrame::RDF;
  } else if (input_frame_str == "FLU") {
    input_frame = ros_sensor_streams::TUMRGBDOfflineStream::InputFrame::FLU;
  } else if (input_frame_str == "FRD") {
    input_frame = ros_sensor_streams::TUMRGBDOfflineStream::InputFrame::FRD;
  } else if (input_frame_str == "RDF_IN_FLU") {
    input_frame = ros_sensor_streams::TUMRGBDOfflineStream::InputFrame::RDF_IN_FLU;
  } else if (input_frame_str == "RDF_IN_FRD") {
    input_frame = ros_sensor_streams::TUMRGBDOfflineStream::InputFrame::RDF_IN_FRD;
  } else {
    ROS_ERROR("Unknown input frame!\n");
    return 1;
  }

  double depth_scale_factor;
  getParamOrFail(pnh, "depth_scale_factor", &depth_scale_factor);

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
                                 TUMRGBDOfflineStream>(nh,
                                                       input_file,
                                                       calib_file,
                                                       "camera",
                                                       camera_world_frame_id,
                                                       camera_frame_id,
                                                       input_frame,
                                                       depth_scale_factor);

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
    Eigen::Quaternionf q;
    Eigen::Vector3f t;
    input->get(&img_id, &time, &rgb, &depth, &q, &t);
    ROS_INFO("Got image %i at time %f", img_id, time);

    cv::Mat1b img_gray;
    cv::cvtColor(rgb, img_gray, cv::COLOR_RGB2GRAY);

    // Process image.
    if (num_imgs == 0) {
      system->randomInit(img_gray.data, time, img_id);
    } else {
      system->trackFrame(img_gray.data, img_id , false, time);
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
