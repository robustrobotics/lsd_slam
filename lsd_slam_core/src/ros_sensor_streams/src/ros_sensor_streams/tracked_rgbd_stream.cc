// Copyright 2016 Massachusetts Institute of Technology

#include "ros_sensor_streams/tracked_rgbd_stream.h"

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <sophus/se3.hpp>

#include <cv_bridge/cv_bridge.h>

#include "ros_sensor_streams/conversions.h"

namespace ros_sensor_streams {

TrackedRGBDStream::TrackedRGBDStream(const std::string& world_frame_id,
                                           ros::NodeHandle& nh,
                                           int queue_size) :
  nh_(nh),
  inited_(false),
  world_frame_id_(world_frame_id),
  live_frame_id_(),
  width_(0),
  height_(0),
  K_(),
  tf_listener_(nullptr),
  tf_buffer_(),
  image_transport_(nullptr),
  rgb_sub_(),
  depth_sub_(),
  info_sub_(),
  sync_(nullptr),
  queue_(queue_size) {
  // Subscribe to topics.
  image_transport::ImageTransport it_(nh_);
  image_transport_.reset(new image_transport::ImageTransport(nh_));
  rgb_sub_.subscribe(*image_transport_, "rgb", 10);
  depth_sub_.subscribe(*image_transport_, "depth", 10);
  info_sub_.subscribe(nh_, "camera_info", 10);

  // Set up synchronizer.
  sync_.reset(new RGBDSynchronizer(RGBDPolicy(10), rgb_sub_, depth_sub_,
                                   info_sub_));
  sync_->registerCallback(boost::bind(&TrackedRGBDStream::rgbdCallback,
                                      this, _1, _2, _3));

  // Set up tf.
  tf_listener_.reset(new tf2_ros::TransformListener(tf_buffer_));

  return;
}

void TrackedRGBDStream::
rgbdCallback(const sensor_msgs::Image::ConstPtr& rgb,
             const sensor_msgs::Image::ConstPtr& depth,
             const sensor_msgs::CameraInfo::ConstPtr& info) {
  ROS_DEBUG("Received RGBD data!");

  // Grab rgbd data.
  cv_bridge::CvImageConstPtr rgb_ptr;
  cv_bridge::CvImageConstPtr depth_ptr;
  rgb_ptr = cv_bridge::toCvCopy(rgb, "bgr8");
  depth_ptr = cv_bridge::toCvCopy(depth, "32FC1");

  assert(rgb_ptr->image.isContinuous());
  assert(depth_ptr->image.isContinuous());

  if (!inited_) {
    live_frame_id_ = rgb_ptr->header.frame_id;

    // Set calibration.
    width_ = rgb_ptr->image.cols;
    height_ = rgb_ptr->image.rows;

    for (int ii = 0; ii < 3; ++ii) {
      for (int jj = 0; jj < 3; ++jj) {
        K_(ii, jj) = info->P[ii*4 + jj];
      }
    }

    inited_ = true;

    ROS_DEBUG("Set RGBD calibration!");
  }

  // Get pose of camera.
  geometry_msgs::TransformStamped tf;
  try {
    // Need to remove leading "/" if it exists.
    std::string rgb_frame_id = rgb->header.frame_id;
    if (rgb_frame_id[0] == '/') {
      rgb_frame_id = rgb_frame_id.substr(1, rgb_frame_id.size()-1);
    }

    tf = tf_buffer_.lookupTransform(world_frame_id_, rgb_frame_id,
                                    ros::Time(rgb->header.stamp),
                                    ros::Duration(1.0/15));
  } catch (tf2::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }

  Sophus::SE3f pose;
  tfToSophusSE3<float>(tf.transform, &pose);

  Frame frame;
  frame.id = rgb->header.seq;
  frame.time = rgb->header.stamp.toSec();
  frame.quat = pose.unit_quaternion();
  frame.trans = pose.translation();
  frame.img = rgb_ptr->image;
  frame.depth = depth_ptr->image;

  queue_.push(frame);

  return;
}

}  // namespace ros_sensor_streams
