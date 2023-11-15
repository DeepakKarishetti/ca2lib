#include <ros/ros.h>

#include <mutex>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "ca2lib/io/ros.h"
#include "ca2lib/plane_extractor.h"
#include "ca2lib/projection.h"
#include "ca2lib/types.h"

void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& msg_);
void updateViewport(int);
void mouseCallback(int, int, int, int, void*);

/* Viewport related variables */
cv::Mat viewport;
cv::Mat range_image;
std::mutex mutex_viewport;
int selection_radius = 10;
cv::Point2i selection_center;
bool roi_set = false;
cv::Point2i roi_center;
int roi_radius;

/* Range image related variables */
float rangeimage_max_range = 10000;
ca2lib::PlaneExtractorLidar extractor;
bool plane_found = false;
std::vector<cv::Point2i> ransac_inliers_pixels;

int main(int argc, char** argv) {
  ros::init(argc, argv, "cloud_plane_estimator");

  ros::NodeHandle nh;

  ros::Subscriber sub_cloud =
      nh.subscribe<sensor_msgs::PointCloud2>("/cloud_in", 10, cloud_cb);

  cv::namedWindow("Viewport");
  cv::setMouseCallback("Viewport", mouseCallback);

  extractor.setRansacParams({300, 0.02, 0.1});

  while (ros::ok()) {
    ros::spinOnce();
    if (!viewport.empty()) cv::imshow("Viewport", viewport);
    updateViewport(cv::waitKey(10));
  }

  return 0;
}

void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& msg_) {
  ca2lib::PointCloudXf cloud = ca2lib::convertRosToPointCloud(msg_);

  std::vector<std::pair<bool, cv::Point2i>> inverse_lut;
  const auto lut = ca2lib::projectLidarLUT(cloud, inverse_lut);

  mutex_viewport.lock();
  range_image = ca2lib::composeChannelImage(cloud, lut, "range",
                                            ca2lib::NormalizationType::MINMAX,
                                            0, rangeimage_max_range);
  mutex_viewport.unlock();

  if (roi_set) {
    // Run RANSAC on patch

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
    if (train_points.size() > 3) {
      extractor.mask() = train_points;
      extractor.setData(&cloud);
      plane_found = extractor.process();
    } else {
      plane_found = false;
    }

    if (plane_found) {
      ransac_inliers_pixels.clear();
      ransac_inliers_pixels.reserve(extractor.inliers().size());
      for (const auto idx : extractor.inliers()) {
        const auto& [valid, p] = inverse_lut[idx];
        if (valid) {
          ransac_inliers_pixels.push_back(p);
        }
      }
    }
  }
  return;
}

void updateViewport(int key_pressed) {
  if (range_image.empty()) return;
  mutex_viewport.lock();
  cv::cvtColor(range_image, viewport, cv::COLOR_GRAY2BGR);

  if (plane_found) {
    // Draw RANSAC inliers
    for (const auto p : ransac_inliers_pixels) {
      viewport.at<cv::Vec3f>(p) = cv::Vec3f(0, 0, 1.);
    }
    std::cerr << "Current plane: " << extractor.plane().transpose()
              << std::endl;
  }

  cv::circle(viewport, selection_center, selection_radius,
             cv::Scalar(125, 125, 125), 1);
  cv::circle(viewport, roi_center, roi_radius, cv::Scalar(0, 255, 0), 1);
  mutex_viewport.unlock();

  switch (key_pressed) {
    case 0x69:  // i
      selection_radius++;
      break;
    case 0x6F:  // o
      selection_radius--;
      break;

    case 0x2B:  // +
      rangeimage_max_range += 1000;
      break;

    case 0x2D:  // -
      rangeimage_max_range = std::max(1000., rangeimage_max_range - 1000.);
      break;
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