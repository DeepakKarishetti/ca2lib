// clang-format off

// Copyright (c) 2023, Robotics Vision and Perception Group

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// clang-format on

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <spdlog/spdlog.h>

#include <filesystem>
#include <fstream>
#include <opencv2/highgui.hpp>

#include "ca2lib/io/ros.h"
#include "ca2lib/parse_command_line.h"
#include "ca2lib/plane_extractor.h"
#include "ca2lib/projection.h"
#include "ca2lib/solver/solver.h"
#include "ca2lib/targets.h"
#include "ca2lib/types.h"

using namespace srrg2_core;

const char* whatdoes = "Calibrate LiDAR-RGB Camera pair.";

cv::Mat viewport;
ca2lib::CameraIntrinsics camera_info;
ca2lib::TargetBase::SharedPtr target_ptr = nullptr;
ca2lib::PointCloudXf cloud;
// Range Image max range
float ri_max_range = 10000;
bool roi_set = false;
int roi_radius;
bool plane_found = false;
cv::Point2i roi_center;
cv::Point2i selection_center;
int selection_radius = 10;
ca2lib::PlaneExtractorLidar extractor_cloud;
ca2lib::PlaneExtractorMonocular extractor_camera;
std::vector<cv::Point> ransac_inlier_pixels;
cv::Mat range_image, camera_image;
ca2lib::Measurements measurement_vect;
ca2lib::Solver solver;
Eigen::Isometry3f camera_T_lidar;
bool solution_found = false;
std::ofstream fout_output;

void data_callback(const sensor_msgs::PointCloud2::ConstPtr&,
                   const sensor_msgs::Image::ConstPtr&);
void updateViewport(int);
void mouseCallback(int, int, int, int, void*);
cv::Mat projectLidar(const cv::Mat& image_rgb,
                     const ca2lib::PointCloudXf& cloud,
                     const Eigen::Isometry3f& camera_T_lidar);

int main(int argc, char** argv) {
  ros::init(argc, argv, "calibrate_lidar_camera");

  ParseCommandLine parser(argv, &whatdoes);
  ArgumentString camera_intrinsics_f(&parser, "i", "intrinsics",
                                     "Camera intrinsics file", "");
  ArgumentString output_f(&parser, "o", "output",
                          "Output calibration file [ends for .yaml or .json]",
                          "lidar_in_camera.yaml");
  ArgumentString target_f(&parser, "t", "target",
                          "YAML file containing target metadata", "");

  ArgumentString topic_cam(&parser, "c", "topic-camera",
                           "sensor_msgs/Image topic for camera",
                           "/camera_right/image_raw");
  ArgumentString topic_cloud(&parser, "l", "topic-cloud",
                             "sensor_msgs/PointCloud2 topic for lidar",
                             "/ouster/points");
  ArgumentString output_planes_file(
      &parser, "p", "output-planes",
      "If set, write on output-file (typically "
      ".txt) the pair <cloud-plane> <camera-plane>",
      "");
  ArgumentString output_transformed_planes_file(
      &parser, "tp", "transformed-planes",
      "If set, write on transformed-planes-file (typically "
      ".txt) the pair <cloud-plane> <camera-plane>",
      "");
  ArgumentFlag use_approx_sync(&parser, "a", "approx-sync",
                               "Use ApproximateTime synchronization policy "
                               "instead of ExactTime policy");
  ArgumentFloat inlier_threshold(
      &parser, "T", "threshold-inlier",
      "Threshold for a measurement to be considered inlier", 3.0f);

  ArgumentInt no_iterations(&parser, "n", "iterations",
                            "Number of solver iterations", 10);
  ArgumentFloat damping_factor(&parser, "d", "damping", "Damping factor", 10.f);
  ArgumentFloat huber_delta(&parser, "u", "huber", "Huber threshold", 0.1f);

  parser.parse();

  if (!camera_intrinsics_f.isSet()) {
    std::cerr << "No camera intrinsics file set.\n"
              << parser.options() << std::endl;
    return -1;
  }

  if (!target_f.isSet()) {
    std::cerr << "No target file set.\n" << parser.options() << std::endl;
    return -1;
  }

  spdlog::info("Reading target config " + target_f.value());
  target_ptr = ca2lib::TargetBase::fromFile(target_f.value());
  extractor_camera.setTarget(target_ptr);

  spdlog::info("Reading camera intrinsics " + camera_intrinsics_f.value());
  camera_info = ca2lib::CameraIntrinsics::load(camera_intrinsics_f.value());
  extractor_camera.setCameraParams(camera_info.K, camera_info.dist_coeffs);

  if (output_planes_file.isSet()) fout_output.open(output_planes_file.value());

  cv::namedWindow("Viewport");
  cv::namedWindow("Camera Image");
  cv::namedWindow("Reprojection");
  cv::setMouseCallback("Viewport", mouseCallback);

  extractor_cloud.setRansacParams({300, 0.02, 0.1});

  ros::NodeHandle nh;

  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_cloud(
      nh, topic_cloud.value(), 10);
  message_filters::Subscriber<sensor_msgs::Image> sub_image(
      nh, topic_cam.value(), 10);

  // Setup solver

  solver.dumping() = damping_factor.value();
  solver.iterations() = no_iterations.value();
  solver.inlierTh() = inlier_threshold.value();
  solver.setMEstimator(std::bind(ca2lib::huber, std::placeholders::_1,
                                 std::placeholders::_2, huber_delta.value()));

  if (use_approx_sync.isSet()) {
    spdlog::info("Using ApproxTime synchronization policy");
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::PointCloud2, sensor_msgs::Image>;
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), sub_cloud,
                                                   sub_image);
    sync.registerCallback(std::bind(&data_callback, std::placeholders::_1,
                                    std::placeholders::_2));
    while (ros::ok()) {
      ros::spinOnce();
      if (!viewport.empty()) {
        cv::Mat cur_image = camera_image.clone();
        cv::resize(cur_image, cur_image, cv::Size(), 0.5, 0.5);
        cv::imshow("Camera Image", cur_image);
        cv::imshow("Viewport", viewport);
      }
      updateViewport(cv::waitKey(10));
    }
  } else {
    spdlog::info("Using ExactTime synchronization policy");
    using SyncPolicy =
        message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2,
                                                  sensor_msgs::Image>;
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), sub_cloud,
                                                   sub_image);
    sync.registerCallback(std::bind(&data_callback, std::placeholders::_1,
                                    std::placeholders::_2));
    while (ros::ok()) {
      ros::spinOnce();
      if (!viewport.empty()) {
        cv::imshow("Camera Image", camera_image);
        cv::imshow("Viewport", viewport);
      }
      updateViewport(cv::waitKey(10));
    }
  }

  ca2lib::CameraLidarExtrinsics camera_lidar_extrinsics(camera_T_lidar);
  camera_lidar_extrinsics.save(output_f.value());

  if (output_transformed_planes_file.isSet()) {
    solver.dumpResult(output_transformed_planes_file.value());
    spdlog::info(
        "To inspect the result compare the planes before and after "
        "calibration:");
    spdlog::info("python3 plot_planes.py --input_file {}",
                 output_planes_file.value());
    spdlog::info("python3 plot_planes.py --input_file {}",
                 output_transformed_planes_file.value());
  }

  return 0;
}

void data_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_,
                   const sensor_msgs::Image::ConstPtr& image_msg_) {
  // Process Cloud
  cloud = ca2lib::convertRosToPointCloud(cloud_msg_);
  ca2lib::InverseLut_t inverse_lut;
  const auto lut = ca2lib::projectLidarLUT(cloud, inverse_lut);

  // Extract data from ROS messages
  range_image = ca2lib::composeChannelImage(
      cloud, lut, "range", ca2lib::NormalizationType::MINMAX, 0, ri_max_range);

  camera_image = cv_bridge::toCvShare(image_msg_, "bgr8")->image;

  // Find target on LiDAR cloud
  if (roi_set) {
    // Create ROI mask
    std::vector<unsigned int> train_points;
    train_points.reserve(cloud.points.size());
    cv::Mat mask = cv::Mat::zeros(range_image.size(), CV_8UC1);
    cv::circle(mask, roi_center, roi_radius, 1, -1);
    // Store masked cloud points using LUT
    for (int r = -roi_radius; r < roi_radius; ++r) {
      for (int c = -roi_radius; c < roi_radius; ++c) {
        const cv::Point2i query(roi_center.x + r, roi_center.y + c);
        if (query.x < 0 or query.x >= mask.cols or query.y < 0 or
            query.y >= mask.rows) {
          continue;
        }
        if (mask.at<unsigned char>(query) > 0) {
          int32_t lut_idx = lut.at<int32_t>(query);
          if (lut_idx == -1) continue;
          train_points.push_back(lut_idx);
        }
      }
    }
    // Run plane extractor
    if (train_points.size() > 3) {
      extractor_cloud.mask() = train_points;
      extractor_cloud.setData(&cloud);
      plane_found = extractor_cloud.process();
    } else
      plane_found = false;
  }

  if (plane_found) {
    ransac_inlier_pixels.clear();
    ransac_inlier_pixels.reserve(extractor_cloud.inliers().size());
    for (const auto idx : extractor_cloud.inliers()) {
      const auto& [valid, p] = inverse_lut[idx];
      if (valid) {
        ransac_inlier_pixels.push_back(p);
      }
    }
  }
}

void updateViewport(int key_pressed) {
  if (range_image.empty() or camera_image.empty()) return;
  cv::cvtColor(range_image, viewport, cv::COLOR_GRAY2BGR);
  if (plane_found) {
    // Draw RANSAC inliers
    for (const auto p : ransac_inlier_pixels) {
      viewport.at<cv::Vec3f>(p) = cv::Vec3f(0, 0, 1.);
    }
  }

  cv::circle(viewport, selection_center, selection_radius,
             cv::Scalar(125, 125, 125), 1);
  cv::circle(viewport, roi_center, roi_radius, cv::Scalar(0, 255, 0), 1);
  switch (key_pressed) {
    case 0x69:  // i
      selection_radius++;
      break;
    case 0x6F:  // o
      selection_radius--;
      break;

    case 0x2B:  // +
      ri_max_range += 1000;
      break;

    case 0x2D:  // -
      ri_max_range = std::max(1000., ri_max_range - 1000.);
      break;

    case 0x0D:  // Enter
      extractor_camera.setData(camera_image);
      if (extractor_camera.process()) {
        spdlog::info("Target detected on camera image");
        std::cerr << "plane_camera="
                  << extractor_camera.plane().normal().transpose() << std::endl;
        std::cerr << "plane_lidar ="
                  << extractor_cloud.plane().normal().transpose() << std::endl;

        if (fout_output.is_open()) {
          const auto& p_cud = extractor_cloud.plane();
          const auto& p_cam = extractor_camera.plane();
          fout_output << p_cud.normal().x() << " " << p_cud.normal().y() << " "
                      << p_cud.normal().z() << " " << p_cud.d();
          fout_output << ", ";
          fout_output << p_cam.normal().x() << " " << p_cam.normal().y() << " "
                      << p_cam.normal().z() << " " << p_cam.d() << "\n";
        }

        cv::Mat frame_detection = camera_image.clone();
        extractor_camera.drawPlane(frame_detection);
        cv::resize(frame_detection, frame_detection, cv::Size(), 0.5, 0.5);
        cv::imshow("Plane Detection", frame_detection);

        ca2lib::Measurement meas;
        meas.from = extractor_cloud.plane();
        meas.to = extractor_camera.plane();
        meas.id = measurement_vect.size();
        measurement_vect.push_back(meas);

        if (measurement_vect.size() > 30) {
          spdlog::info("Solving extrinsics camera_T_lidar");
          solver.estimate() = Eigen::Isometry3f::Identity();
          solver.measurements() = measurement_vect;
          solver.compute();

          std::cerr << solver.stats() << std::endl;

          camera_T_lidar = solver.estimate();
          std::cerr << "camera_T_lidar:\n"
                    << camera_T_lidar.matrix() << std::endl;
          if (solver.stats().back().status == ca2lib::IterationStat::SolverStatus::Success) {
            solution_found = true;
            ROS_WARN("Solver: solution found");
          } else {
            ROS_ERROR("Solver: solution not found");
          }
        }
      }
      break;

    default:
      break;
  }

  // Display reprojection
  if (solution_found) {
    cv::Mat cloud_reprojected = projectLidar(camera_image, cloud, camera_T_lidar);
    cv::resize(cloud_reprojected, cloud_reprojected, cv::Size(), 0.5, 0.5);
    cv::imshow("Reprojection", cloud_reprojected);
  }
}

void mouseCallback(int event, int x, int y, int flags, void*) {
  selection_center = cv::Point2i(x, y);
  if (event == cv::EVENT_LBUTTONDOWN) {
    roi_set = true;
    roi_center = selection_center;
    roi_radius = selection_radius;
  }
}

cv::Mat projectLidar(const cv::Mat& image_rgb,
                     const ca2lib::PointCloudXf& cloud,
                     const Eigen::Isometry3f& camera_T_lidar) {
  cv::Mat ret = image_rgb.clone();
  ca2lib::InverseLut_t inv_lut;
  std::cerr << "cloud size| " << cloud.points.size() << " | camera_T_lidar : \n"
            << camera_T_lidar.matrix() << std::endl;
  cv::Mat lut = ca2lib::projectPinholeLUT(cloud, inv_lut, ret.size(),
                                          camera_info, camera_T_lidar);

  for (unsigned int i = 0; i < cloud.points.size(); ++i) {
    if (inv_lut[i].first)
      cv::circle(ret, inv_lut[i].second, 3, {0, 0, 255}, -1);
  }
  return ret;
}