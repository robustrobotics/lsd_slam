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


#include "IOWrapper/ROS/ROSImageStreamThread.h"
#include "IOWrapper/ROS/ROSOutput3DWrapper.h"
#include "IOWrapper/ROS/rosReconfigure.h"

#include <tf2_ros/transform_listener.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <geometry_msgs/TransformStamped.h>

#include <cv_bridge/cv_bridge.h>

#include <eigen_conversions/eigen_msg.h>

using namespace lsd_slam;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                        sensor_msgs::CameraInfo,
                                                        sensor_msgs::Image,
                                                        sensor_msgs::CameraInfo> SyncPolicy;

void groundTruthCallback(const sensor_msgs::Image::ConstPtr& rgb,
                         const sensor_msgs::CameraInfo::ConstPtr& rgb_info,
                         const sensor_msgs::Image::ConstPtr& depth,
                         const sensor_msgs::CameraInfo::ConstPtr& depth_info,
                         lsd_slam::LiveSLAMWrapper* driver,
                         message_filters::Subscriber<sensor_msgs::Image>* rgb_sub,
                         message_filters::Subscriber<sensor_msgs::CameraInfo>* rgb_info_sub,
                         message_filters::Subscriber<sensor_msgs::Image>* depth_sub,
                         message_filters::Subscriber<sensor_msgs::CameraInfo>* depth_info_sub) {
  double time = rgb->header.stamp.toSec();

  // Set initial pose to ground truth pose.
  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener listener(buffer);
  geometry_msgs::TransformStamped tf, tf2;

  // TODO(wng): What happens if these frame ids change?  Any way to remap them?
  // NOTE: Desired time should be rgb->header.stamp, but ros complains that this
  // is earlier than the earliest available tf. For now just use most recent tf.
  tf = buffer.lookupTransform("world", "openni_rgb_optical_frame", ros::Time(0),
                              ros::Duration(1.0));
  tf2 = buffer.lookupTransform("openni_rgb_optical_frame", "openni_rgb_frame", ros::Time(0),
                              ros::Duration(1.0));

  Eigen::Affine3d pose_se3;
  tf::transformMsgToEigen(tf.transform, pose_se3);

  Eigen::Affine3d flu_to_optical;
  tf::transformMsgToEigen(tf2.transform, flu_to_optical);

  pose_se3 = flu_to_optical * pose_se3;

  Sophus::Sim3f pose_sim3;
  pose_sim3.quaternion() = pose_se3.rotation().cast<float>();
  pose_sim3.translation() = pose_se3.translation().cast<float>();

  ROS_INFO("Initializing orientation to qw = %f, qx = %f, qy = %f, qz = %f",
           pose_sim3.quaternion().w(), pose_sim3.quaternion().x(),
           pose_sim3.quaternion().y(), pose_sim3.quaternion().z());
  ROS_INFO("Initializing translation to tx = %f, ty = %f, tz = %f",
           pose_sim3.translation()(0), pose_sim3.translation()(1),
           pose_sim3.translation()(2));

  // Convert to OpenCV images.
  cv_bridge::CvImagePtr rgb_cv = cv_bridge::toCvCopy(rgb, rgb->encoding);
  cv_bridge::CvImagePtr depth_cv = cv_bridge::toCvCopy(depth, "32FC1");

  // Initialize!
  driver->initializeFromGTDepth(time, rgb_cv->image, depth_cv->image, pose_sim3);

  // Unsubscribe.
  rgb_sub->unsubscribe();
  rgb_info_sub->unsubscribe();
  depth_sub->unsubscribe();
  depth_info_sub->unsubscribe();

  return;
}

int main( int argc, char** argv )
{
	ros::init(argc, argv, "LSD_SLAM");

	dynamic_reconfigure::Server<lsd_slam_core::LSDParamsConfig> srv(ros::NodeHandle("~"));
	srv.setCallback(dynConfCb);

	dynamic_reconfigure::Server<lsd_slam_core::LSDDebugParamsConfig> srvDebug(ros::NodeHandle("~Debug"));
	srvDebug.setCallback(dynConfCbDebug);

	packagePath = ros::package::getPath("lsd_slam_core")+"/";

	InputImageStream* inputStream = new ROSImageStreamThread();

	std::string calibFile;
	if(ros::param::get("~calib", calibFile))
	{
		ros::param::del("~calib");
		inputStream->setCalibration(calibFile);
	}
	else
    inputStream->setCalibration("");

  Output3DWrapper* outputWrapper = new ROSOutput3DWrapper(inputStream->width(), inputStream->height());
  LiveSLAMWrapper slamNode(inputStream, outputWrapper);

  // Initialize using a Kinect frame.
  ROS_INFO("Initializing from ground truth...");

  ros::NodeHandle nh;

  message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "image", 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> rgb_info(nh, "camera_info", 1);

  message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "depth", 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> depth_info(nh, "depth_camera_info", 1);

  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), rgb_sub, rgb_info,
                                                 depth_sub, depth_info);
  sync.registerCallback(boost::bind(&groundTruthCallback, _1, _2, _3, _4, &slamNode,
                                    &rgb_sub, &rgb_info, &depth_sub, &depth_info));

  while (!slamNode.inited()) {
    // Wait until the driver is initialized.
    ros::spinOnce();
  }

  ROS_INFO("Initialized!");


  inputStream->run();
  slamNode.Loop();



	if (inputStream != nullptr)
		delete inputStream;
	if (outputWrapper != nullptr)
		delete outputWrapper;

	return 0;
}
