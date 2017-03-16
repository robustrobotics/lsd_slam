// Copyright 2016 Massachusetts Institute of Technology

#pragma once

#include <memory>
#include <string>

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf2_ros/transform_listener.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <geometry_msgs/TransformStamped.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core/core.hpp>

#include <ros_sensor_streams/thread_safe_queue.h>

namespace ros_sensor_streams {

/**
 * \brief Class that represents an input stream of tracked RGBD images.
 *
 * Assumes the RGB and depth images are registered (i.e. the RGB and depth
 * pixels are aligned).
 *
 * Designed for use in a nodelet - thus there is no ros::spin() call.
 */
class TrackedRGBDStream final  {
 public:
  // Convenience typedefs.
  typedef message_filters::sync_policies::
  ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,
                  sensor_msgs::CameraInfo> RGBDPolicy;
  typedef message_filters::Synchronizer<RGBDPolicy> RGBDSynchronizer;

  /**
   * @brief Struct to hold image and pose data.
   */
  struct Frame {
    uint32_t id; // Image ID.
    double time; // Timestamp.
    Eigen::Quaternionf quat; // Orientation as quaternion.
    Eigen::Vector3f trans; // Translsation.
    cv::Mat3b img; // RGB image.
    cv::Mat depth; // Depth.
  };

  TrackedRGBDStream(const std::string& world_frame_id, ros::NodeHandle& nh,
                       int queue_size = 8);

  ~TrackedRGBDStream() = default;

  TrackedRGBDStream(const TrackedRGBDStream& rhs) = delete;
  TrackedRGBDStream& operator=(const TrackedRGBDStream& rhs) = delete;

  TrackedRGBDStream(const TrackedRGBDStream&& rhs) = delete;
  TrackedRGBDStream& operator=(const TrackedRGBDStream&& rhs) = delete;

  ThreadSafeQueue<Frame>& queue() { return queue_; }

  /**
   * \brief Returns true if stream is initialized.
   */
  bool inited() {
    return inited_;
  }

  /**
   * \brief Get the image width.
   */
  int width() {
    return width_;
  }

  /**
   * \brief Get the image height.
   */
  int height() {
    return height_;
  }

  /**
   * \brief Return the camera intrinsic matrix.
   */
  const Eigen::Matrix3f& K() {
    return K_;
  }

  /**
   * \brief Return id of the world frame.
   */
  const std::string& world_frame_id() {
    return world_frame_id_;
  }

  /**
   * \brief Return id of the live frame (i.e. the pose of the camera).
   */
  const std::string& live_frame_id() {
    return live_frame_id_;
  }

 private:
  void rgbdCallback(const sensor_msgs::Image::ConstPtr& rgb,
                    const sensor_msgs::Image::ConstPtr& depth,
                    const sensor_msgs::CameraInfo::ConstPtr& info);

  ros::NodeHandle& nh_;

  bool inited_;
  std::string world_frame_id_;
  std::string live_frame_id_;
  int width_;
  int height_;
  Eigen::Matrix3f K_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  tf2_ros::Buffer tf_buffer_;

  // Will use a combination of message_filters and image_transport to
  // synchronize rgb, depth, and camera info topics.
  std::shared_ptr<image_transport::ImageTransport> image_transport_;
  image_transport::SubscriberFilter rgb_sub_;
  image_transport::SubscriberFilter depth_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_;

  std::shared_ptr<RGBDSynchronizer> sync_;

  ThreadSafeQueue<Frame> queue_;
};

}  // namespace ros_sensor_streams
