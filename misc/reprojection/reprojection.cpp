#ifndef UTILS_SRC_REPROJECTION_REPROJECTION_CPP
#define UTILS_SRC_REPROJECTION_REPROJECTION_CPP

#include "reprojection.h"

namespace utils
{

cv::Mat Reprojection::convertProjection(const std::shared_ptr<open3d::geometry::PointCloud>& input_pointcloud,
                                        std::vector<std::pair<bool, cv::Point2i>>& inverse_lookup_table,
                                        const cv::Size& image_size,
                                        const Eigen::Isometry3f& camera_T_lidar)
{
    if (!inverse_lookup_table.empty()) {
        inverse_lookup_table.clear();
    }
    inverse_lookup_table.resize(input_pointcloud->points_.size());

    std::vector<int32_t> points_idx;
    std::vector<cv::Vec3f> points_cv;
    for (unsigned int i=0; i<input_pointcloud->points_.size(); i++)
    {
        // const Eigen::Vector3f& pt = input_pointcloud->points_[i];
        const Eigen::Vector3f pt(input_pointcloud->points_[i].x(), input_pointcloud->points_[i].y(), input_pointcloud->points_[i].z());
        Eigen::Vector3f pt_cam = camera_T_lidar * pt;
        if (pt_cam.z() < 0)
        {
            inverse_lookup_table[i] = {false, cv::Point2i(0, 0)};
            continue;
        }

        points_idx.push_back(i);
        points_cv.push_back(cv::Vec3f(pt_cam.x(), pt_cam.y(), pt_cam.z()));
    }

    //! DEBUG: **************************************** DEBUG ****************************************
    if (reprojectionInputs_.camera_intrinsics.K.empty()) {
        std::cout << "Camera matrix K is empty." << std::endl;
        return cv::Mat::zeros(image_size, CV_32S);
    }

    if (reprojectionInputs_.camera_intrinsics.K.rows != 3 || reprojectionInputs_.camera_intrinsics.K.cols != 3) {
        std::cout << "Camera matrix K must be 3x3. Current size: " 
                  << reprojectionInputs_.camera_intrinsics.K.rows << "x" 
                  << reprojectionInputs_.camera_intrinsics.K.cols << std::endl;
        return cv::Mat::zeros(image_size, CV_32S);
    }

    if (reprojectionInputs_.camera_intrinsics.dist_coeffs.empty()) {
        std::cout << "Distortion coefficients are empty." << std::endl;
        return cv::Mat::zeros(image_size, CV_32S);
    }

    if (reprojectionInputs_.camera_intrinsics.dist_coeffs.cols != 1 && reprojectionInputs_.camera_intrinsics.dist_coeffs.rows != 1) {
        std::cout << "Distortion coefficients must be a vector. Current size: " 
                  << reprojectionInputs_.camera_intrinsics.dist_coeffs.rows << "x" 
                  << reprojectionInputs_.camera_intrinsics.dist_coeffs.cols << std::endl;
        return cv::Mat::zeros(image_size, CV_32S);
    }
    //! DEBUG: **************************************** DEBUG ****************************************

    // cv::Mat rvec = cv::Mat::zeros(3, 1, CV_32F);
    // cv::Mat tvec = cv::Mat::zeros(3, 1, CV_32F);
    cv::Mat rvec(3, 1, CV_32F);
    cv::Mat tvec(3, 1, CV_32F);
    rvec = 0;
    tvec = 0;

    std::vector<cv::Point2f> points_2d;
    try
    {
        if (reprojectionInputs_.camera_intrinsics.dist_coeffs.cols == 4) 
        {
            cv::fisheye::projectPoints(points_cv, points_2d, rvec, tvec,
                                       reprojectionInputs_.camera_intrinsics.K,
                                       reprojectionInputs_.camera_intrinsics.dist_coeffs);
        }
        else
        {
            DEBUG_PRINT("before project points");
            cv::projectPoints(points_cv, rvec, tvec, reprojectionInputs_.camera_intrinsics.K,
                              reprojectionInputs_.camera_intrinsics.dist_coeffs, points_2d);
            DEBUG_PRINT("after project points");
        }
    }
    catch (const cv::Exception& e)
    {
        std::cout << "Error in projectPoints: " << e.what() << std::endl;
        return cv::Mat::zeros(image_size, CV_32S);
    }

    DEBUG_PRINT("Before lut init");
    cv::Mat lut = cv::Mat_<int32_t>(image_size);
    lut = -1;
    // cv::Mat lut(image_size, CV_32SC1, cv::Scalar(-1));
    for (unsigned int i = 0; i < points_2d.size(); ++i) {
        if (points_2d[i].x < 0 || points_2d[i].x >= image_size.width ||
            points_2d[i].y < 0 || points_2d[i].y >= image_size.height) {
            inverse_lookup_table[points_idx[i]] = {false, cv::Point2i(0, 0)};
            continue;
        }
        cv::Point2i index = cv::Point2i(points_2d[i].x, points_2d[i].y);
        lut.at<int32_t>(index) = points_idx[i];
        inverse_lookup_table[points_idx[i]] = {true, index};
    }

    return lut;
}

cv::Mat Reprojection::projectLidarToImage(const cv::Mat& input_image, 
                                          const std::shared_ptr<open3d::geometry::PointCloud>& input_pointcloud,
                                          const Eigen::Isometry3f& camera_T_lidar)
{
    DEBUG_PRINT("Projecting Lidar to Image");
    cv::Mat reprojected_image = input_image.clone();
    std::vector<std::pair<bool, cv::Point2i>> inverse_lookup_table;
    Reprojection::convertProjection(input_pointcloud, inverse_lookup_table, input_image.size(), camera_T_lidar);

    for (unsigned int i=0; i<input_pointcloud->points_.size(); i++)
    {
        if (inverse_lookup_table[i].first)
        {
            cv::circle(reprojected_image, inverse_lookup_table[i].second, 3, {0, 0, 255}, -1);
        }
    }
    return reprojected_image;
}

void Reprojection::loadParams()
{
    reprojectionInputs_.loadInputs();
    reprojectionInputs_.visualizeInputs();
}

Reprojection::Reprojection(const Inputs& inputs_) : reprojectionInputs_(inputs_)
{
    Reprojection::loadParams();
    cv::Mat reprojected_img = Reprojection::projectLidarToImage(reprojectionInputs_.image, 
                                                                reprojectionInputs_.pointcloud, 
                                                                reprojectionInputs_.camera_T_lidar);
    cv::imshow("Reprojected Image", reprojected_img);
    cv::waitKey(0);
}

} //! end namespace utils

#endif //! UTILS_SRC_REPROJECTION_REPROJECTION_CPP
