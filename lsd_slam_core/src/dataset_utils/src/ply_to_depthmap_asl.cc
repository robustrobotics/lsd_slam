/**
 * Copyright 2017 Massachusetts Institute of Technology
 *
 * @file ply_to_depthmap_asl.cc
 * @author W. Nicholas Greene
 * @date 2017-07-27 19:19:50 (Thu)
 */

#include <stdio.h>

#include <string>
#include <iostream>
#include <limits>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <pcl/io/ply_io.h>

#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "dataset_utils/utils.h"
#include "dataset_utils/asl/types.h"
#include "dataset_utils/asl/dataset.h"

namespace bfs = boost::filesystem;
namespace bpo = boost::program_options;

namespace du = dataset_utils;
namespace dua = dataset_utils::asl;

/**
 * @brief Compute pose of camera in world frame.
 */
void getCameraInWorld(const dua::Dataset<dua::FileData>& cam_data,
                      const dua::Dataset<dua::PoseData>& pose_data,
                      const std::vector<std::size_t>& cam_idxs,
                      const std::vector<std::size_t>& pose_idxs,
                      std::vector<Eigen::Quaterniond>* q_cam_in_world,
                      std::vector<Eigen::Vector3d>* t_cam_in_world) {
  // Extract transform of pose sensor in body frame.
  Eigen::Matrix<double, 4, 4, Eigen::RowMajor> T_psensor_in_body;
  du::readMatrix(pose_data.metadata(), "T_BS", 4, 4, T_psensor_in_body.data());
  Eigen::Matrix4d T_body_in_psensor(T_psensor_in_body.inverse());
  Eigen::Quaterniond q_body_in_psensor(T_body_in_psensor.block<3, 3>(0, 0));
  Eigen::Vector3d t_body_in_psensor(T_body_in_psensor.block<3, 1>(0, 3));

  // Extract transform of camera in body frame.
  Eigen::Matrix<double, 4, 4, Eigen::RowMajor> T_cam_in_body;
  du::readMatrix(cam_data.metadata(), "T_BS", 4, 4, T_cam_in_body.data());
  Eigen::Quaterniond q_cam_in_body(T_cam_in_body.block<3, 3>(0, 0));
  Eigen::Vector3d t_cam_in_body(T_cam_in_body.block<3, 1>(0, 3));

  // Extract poses of camera in world.
  q_cam_in_world->resize(cam_idxs.size());
  t_cam_in_world->resize(cam_idxs.size());
  for (int ii = 0; ii < cam_idxs.size(); ++ii) {
    Eigen::Quaterniond q_psensor_in_world(pose_data[pose_idxs[ii]].quat);
    Eigen::Vector3d t_psensor_in_world(pose_data[pose_idxs[ii]].trans);

    Eigen::Quaterniond q_body_in_world(q_psensor_in_world * q_body_in_psensor);
    Eigen::Vector3d t_body_in_world(q_psensor_in_world * t_body_in_psensor + t_psensor_in_world);

    (*q_cam_in_world)[ii] = q_body_in_world * q_cam_in_body;
    (*t_cam_in_world)[ii] = q_body_in_world * t_cam_in_body + t_body_in_world;
  }

  return;
}

/**
 * @brief Project point cloud into camera to generate depthmap.
 */
void projectPointCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                       int height, int width, const Eigen::Matrix3f& K,
                       const Eigen::Quaterniond& q_world_to_cam,
                       const Eigen::Vector3d& t_world_to_cam,
                       float fill_radius,
                       cv::Mat1f* depthmap) {
  if (depthmap->empty()) {
    depthmap->create(height, width);
  }
  *depthmap = std::numeric_limits<float>::quiet_NaN();

  cv::Rect valid_region(0, 0, width, height);
  Eigen::Vector3d fill_offset(fill_radius, fill_radius, 0.0f);
  for (int ii = 0; ii < cloud.points.size(); ++ii) {
    Eigen::Vector3d p_world(cloud.points[ii].x, cloud.points[ii].y, cloud.points[ii].z);
    Eigen::Vector3d p_cam(q_world_to_cam * p_world + t_world_to_cam);
    float new_depth = p_cam(2);

    if (new_depth <= 0.0f) {
      // Point is behind camera.
      continue;
    }

    // We need to give some volume to each point so that occluded structure
    // doesn't corrupt the depthmap. Assume that each point corresponds to a
    // fronto-parallel plane with radius <fill_radius>.
    Eigen::Vector3d u_hom_top_left(K.cast<double>() * (p_cam - fill_offset));
    u_hom_top_left /= u_hom_top_left(2);

    int u_top_left = u_hom_top_left(0) + 0.5f;
    int v_top_left = u_hom_top_left(1) + 0.5f;

    Eigen::Vector3d u_hom_bot_right(K.cast<double>() * (p_cam + fill_offset));
    u_hom_bot_right /= u_hom_bot_right(2);

    int u_bot_right = u_hom_bot_right(0) + 0.5f;
    int v_bot_right = u_hom_bot_right(1) + 0.5f;

    cv::Rect fill(u_top_left, v_top_left,
                  u_bot_right - u_top_left,
                  v_bot_right - v_top_left);

    fill = fill & valid_region; // Intersect with valid region.

    for (int vi = fill.tl().y; vi <= fill.br().y; ++vi) {
      for (int ui = fill.tl().x; ui <= fill.br().x; ++ui) {
        if ((ui < 0.0f) || (ui >= width - 1) || (vi < 0.0f) || (vi >= height - 1)) {
          // Point projects out of bounds.
          continue;
        }

        float old_depth = (*depthmap)(vi, ui);
        if (std::isnan(old_depth) || (new_depth < old_depth)) {
          (*depthmap)(vi, ui) = new_depth;
        }
      }
    }
  }

  return;
}


/**
 * @brief Parse commandline arguments using boost::program_options.
 */
bool parseArgs(int argc, char *argv[], bpo::variables_map* args) {
  bpo::options_description options("Usage");
  options.add_options()
      ("help,h", "Print help message.")
      ("ply", bpo::value<std::string>()->required(), "Input PLY file.")
      ("camera", bpo::value<std::string>()->required(), "Camera images ASL folder.")
      ("pose", bpo::value<std::string>()->required(), "Pose ASL folder (pose wrt to world).")
      ("output", bpo::value<std::string>()->required(), "Output directory.")
      ("max_diff", bpo::value<float>()->default_value(0.02f), "Maximum time difference to associate images and poses.")
      ("depth_scale_factor", bpo::value<float>()->default_value(1000.0f), "Desired conversion of metric depth to depthmap value.")
      ("fill_radius", bpo::value<float>()->default_value(0.01f), "Radius to assign to each point to fill holes in depthmap.");

  bpo::positional_options_description poptions;
  poptions.add("ply", 1);
  poptions.add("camera", 1);
  poptions.add("pose", 1);
  poptions.add("output", 1);

  try {
    // Parse the command line.
    bpo::store(bpo::command_line_parser(argc, argv).
               options(options).positional(poptions).run(), *args);

    if (args->count("help") > 0) {
      // Print help message.
      printf("This is a utility to convert a PLY file into a set of (undistorted) depthmaps.");
      std::cout << options << std::endl;
      return false;
    }

    bpo::notify(*args); // Raise any errors.
  } catch (bpo::required_option& e) {
    fprintf(stderr, "Error: %s\n", e.what());
    return false;
  } catch (bpo::error& e) {
    fprintf(stderr, "Error: %s\n", e.what());
    return false;
  } catch (std::exception& e) {
    fprintf(stderr, "Error: %s\n", e.what());
    return false;
  } catch(...) {
    fprintf(stderr, "Unknown Error.");
    return false;
  }

  return true;
}

int main(int argc, char *argv[]) {
  // Parse commandline arguments.
  bpo::variables_map args;
  if (!parseArgs(argc, argv, &args)) {
    return 1;
  }

  std::string ply = args["ply"].as<std::string>();
  std::string camera = args["camera"].as<std::string>();
  std::string pose = args["pose"].as<std::string>();
  std::string output = args["output"].as<std::string>();
  float max_diff = args["max_diff"].as<float>();
  float depth_scale_factor = args["depth_scale_factor"].as<float>();
  float fill_radius = args["fill_radius"].as<float>();

  printf("PlyFile: %s\n", ply.c_str());
  printf("CameraASL: %s\n", camera.c_str());
  printf("PoseASL: %s\n", pose.c_str());
  printf("OutputDir: %s\n", output.c_str());

  // Create output directory.
  bfs::path output_dir(output);
  if (bfs::exists(output_dir)) {
    fprintf(stderr, "Output directory %s already exists!\n", output.c_str());
    return 1;
  }

  bfs::create_directory(output_dir);

  // Load ply file as PCL point cloud.
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PLYReader ply_reader;
  ply_reader.read(ply, cloud);

  // Load datasets.
  dua::Dataset<dua::FileData> cam_data(camera);
  dua::Dataset<dua::PoseData> pose_data(pose);

  // Associate image timestamps with pose timestamps.
  std::vector<std::size_t> cam_idxs, pose_idxs;
  auto diff = [](const dua::FileData& x, const dua::PoseData& y) {
    double tx = static_cast<double>(x.timestamp) * 1e-9;
    double ty = static_cast<double>(y.timestamp) * 1e-9;
    return std::fabs(tx - ty);
  };
  du::associate(cam_data.data(), pose_data.data(), &cam_idxs, &pose_idxs, diff,
                max_diff);

  // Compute pose of camera in world frame.
  std::vector<Eigen::Quaterniond> q_cam_in_world(cam_idxs.size());
  std::vector<Eigen::Vector3d> t_cam_in_world(cam_idxs.size());
  getCameraInWorld(cam_data, pose_data, cam_idxs, pose_idxs,
                   &q_cam_in_world, &t_cam_in_world);

  // Extract some camera info.
  int width = cam_data.metadata()["resolution"][0].as<int>();
  int height = cam_data.metadata()["resolution"][1].as<int>();
  float fu = cam_data.metadata()["intrinsics"][0].as<float>();
  float fv = cam_data.metadata()["intrinsics"][1].as<float>();
  float cu = cam_data.metadata()["intrinsics"][2].as<float>();
  float cv = cam_data.metadata()["intrinsics"][3].as<float>();
  float k1 = cam_data.metadata()["distortion_coefficients"][0].as<float>();
  float k2 = cam_data.metadata()["distortion_coefficients"][1].as<float>();
  float p1 = cam_data.metadata()["distortion_coefficients"][2].as<float>();
  float p2 = cam_data.metadata()["distortion_coefficients"][3].as<float>();
  float k3 = 0.0f;

  Eigen::Matrix3f K(Eigen::Matrix3f::Identity());
  K(0, 0) = fu;
  K(0, 2) = cu;
  K(1, 1) = fv;
  K(1, 2) = cv;

  cv::Mat Kcv;
  eigen2cv(K, Kcv);

  std::vector<float> D{k1, k2, p1, p2, k3};

  // Create data directory.
  bfs::path data_dir(output_dir / bfs::path("data"));
  bfs::create_directory(data_dir);

  // Loop over camera poses and generate depthmap and save.
  for (int ii = 0; ii < cam_idxs.size(); ++ii) {
    printf("Depthmap %i/%zu...\n", ii, cam_idxs.size());

    // Grab undistorted image for debugging.
    bfs::path imgpath = bfs::path(cam_data.path()) / bfs::path("/data/") / bfs::path(cam_data[cam_idxs[ii]].filename);
    cv::Mat1b img_raw = cv::imread(imgpath.string(), cv::IMREAD_GRAYSCALE);
    cv::Mat1b img;
    cv::undistort(img_raw, img, Kcv, D);
    cv::imshow("img", img);
    cv::waitKey(1);

    // Project point cloud into camera.
    Eigen::Quaterniond q_world_in_cam(q_cam_in_world[ii].inverse());
    Eigen::Vector3d t_world_in_cam(-(q_cam_in_world[ii].inverse() * t_cam_in_world[ii]));

    cv::Mat1f depthmap(height, width, std::numeric_limits<float>::quiet_NaN());
    projectPointCloud(cloud, height, width, K, q_world_in_cam, t_world_in_cam,
                      fill_radius, &depthmap);

    // Convert depthmap to uint16_t and save as png.
    cv::Mat_<uint16_t> depthmap16(height, width, static_cast<uint16_t>(0));
    for (int ii = 0; ii < height; ++ii) {
      for (int jj = 0; jj < width; ++jj) {
        float depth = depthmap(ii, jj);
        if (std::isnan(depth)) {
          continue;
        }

        float scaled_depth = depth * depth_scale_factor + 0.5f;
        if (scaled_depth > std::numeric_limits<uint16_t>::max()) {
          printf("WARNING OVERFLOW DETECTED!\n");
        }
        depthmap16(ii, jj) = scaled_depth;
      }
    }

    // Save image.
    bfs::path img_fname = data_dir / bfs::path(cam_data[cam_idxs[ii]].filename);
    img_fname.replace_extension("png");
    cv::imwrite(img_fname.string(), depthmap16);

    cv::imshow("depthmap", depthmap16);
  }

  // Write yaml file.
  printf("Writing yaml...\n");
  YAML::Node depthmap_node = cam_data.metadata();
  depthmap_node["sensor_type"] = "depth camera";
  depthmap_node["comment"] = "Converted depthmaps from pointcloud";
  depthmap_node["distortion_coefficients"][0] = 0;
  depthmap_node["distortion_coefficients"][1] = 0;
  depthmap_node["distortion_coefficients"][2] = 0;
  depthmap_node["distortion_coefficients"][3] = 0;
  depthmap_node["distortion_coefficients"][4] = 0;
  depthmap_node["depth_scale_factor"] = depth_scale_factor;

  std::ofstream yout((output_dir / bfs::path("sensor.yaml")).string());
  yout << depthmap_node;

  // Write data.csv.
  printf("Writing CSV...\n");
  std::ofstream csvout((output_dir / bfs::path("data.csv")).string());
  for (int ii = 0; ii < cam_idxs.size(); ++ii) {
    bfs::path img_fname = bfs::path(cam_data[cam_idxs[ii]].filename);
    img_fname.replace_extension("png");
    csvout << cam_data[cam_idxs[ii]].timestamp << "," << img_fname.string() << std::endl;
  }

  printf("Done.");

  return 0;
}
