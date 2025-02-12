#ifndef UTILS_SRC_REPROJECTION_REPROJECTION_UTILS_H
#define UTILS_SRC_REPROJECTION_REPROJECTION_UTILS_H

#include "cnpy.h"
#include <iostream>
#include <fstream>
#include <utility>
#include <map>
#include <vector>
#include <filesystem>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include "open3d/Open3D.h"


namespace utils
{

inline void eigenMatrixToYamlNode(YAML::Node& node, const Eigen::Isometry3f& isometry)
{
    for (int r = 0; r < 3; ++r)
    {
        for (int c = 0; c < 4; ++c)
        {
            node.push_back(isometry.matrix()(r, c));
        }
    }   
}


struct CameraIntrinsics
{
    cv::Mat K;
    cv::Mat dist_coeffs;
    cv::Mat rvecs, tvecs;

    void printValues()
    {
        std::cout << "Camera Intrinsics (K):\n" << K << std::endl;
        std::cout << "Distortion Coefficients:\n" << dist_coeffs << std::endl;

        std::cout << "Rotation Vectors (rvecs):\n";
        for (int i = 0; i < rvecs.rows; ++i) {
            std::cout << rvecs.row(i) << std::endl;
        }

        std::cout << "Translation Vectors (tvecs):\n";
        for (int i = 0; i < tvecs.rows; ++i) {
            std::cout << tvecs.row(i) << std::endl;
        }
    }

    void save(const std::string& f) const
    {
        std::filesystem::path filename = f;
        if (filename.extension() == ".yaml")
        {
            YAML::Node conf;
            YAML::Node storage_K;
            storage_K.push_back(K.at<double>(0, 0));
            storage_K.push_back(K.at<double>(1, 1));
            storage_K.push_back(K.at<double>(0, 2));
            storage_K.push_back(K.at<double>(1, 2));
            YAML::Node storage_D;
            for (int i = 0; i < dist_coeffs.cols; ++i)
            {
                storage_D.push_back(dist_coeffs.at<double>(0, i));
            }
            conf["K"] = storage_K;
            conf["dist_coeffs"] = storage_D;
            YAML::Emitter emitter;
            emitter << conf;
            std::ofstream fout(f);
            fout << emitter.c_str();
            return;
        }
        else if (filename.extension() == ".json")
        {
            throw std::runtime_error("JSON configurations are currently not supported");
        }
        else
        {
            throw std::runtime_error("Format configuration is currently not supported");
        }
        }

    void load(const std::string& f)
    {
        std::filesystem::path filename = f;
        if (filename.extension() == ".yaml")
        {
            const auto config = YAML::LoadFile(f);
            K = cv::Mat::eye(3, 3, CV_64F);
            K.at<double>(0, 0) = config["K"][0].as<double>();
            K.at<double>(1, 1) = config["K"][1].as<double>();
            K.at<double>(0, 2) = config["K"][2].as<double>();
            K.at<double>(1, 2) = config["K"][3].as<double>();
            dist_coeffs = cv::Mat(1, config["dist_coeffs"].size(), CV_64F);
            for (int i = 0; i < dist_coeffs.cols; ++i) {
                dist_coeffs.at<double>(0, i) = config["dist_coeffs"][i].as<double>();
            }
        }
        else if (filename.extension() == ".json")
        {
            throw std::runtime_error("JSON configurations are currently not supported");
        }
        else
        {
            throw std::runtime_error("Format configuration is currently not supported");
        }
    }

    CameraIntrinsics() = default;
};


struct CameraLidarExtrinsics
{
    Eigen::Isometry3f lidar_in_camera;
    Eigen::Isometry3f camera_in_lidar;

    void printValues()
    {
        std::cout << "Lidar in Camera:\n" << lidar_in_camera.matrix() << std::endl;
        std::cout << "Camera in Lidar:\n" << camera_in_lidar.matrix() << std::endl;
    }

    void load(const std::string& f)
    {
        std::filesystem::path filename = f;
        if (filename.extension() == ".yaml") {
            const auto config = YAML::LoadFile(f);

            for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 4; c++) {
                lidar_in_camera.matrix()(r, c) =
                    config["lidar_in_camera"][r * 4 + c].as<double>();
            }
            }

            camera_in_lidar = lidar_in_camera.inverse();

        }
        else if (filename.extension() == ".json")
        {
            throw std::runtime_error("JSON configurations are currently not supported");
        }
        else
        {
            throw std::runtime_error("Format configuration is currently not supported");
        }
    }

    CameraLidarExtrinsics() = default;

    CameraLidarExtrinsics(Eigen::Isometry3f lidar_in_camera_) : lidar_in_camera(lidar_in_camera_), camera_in_lidar(lidar_in_camera.inverse()) {}
};


class Inputs
{
    public:
        cv::Mat image;
        std::shared_ptr<open3d::geometry::PointCloud> pointcloud;
        Eigen::Isometry3f camera_T_lidar;
        CameraIntrinsics camera_intrinsics;
        CameraLidarExtrinsics camera_lidar_extrinsics;

    private:
        void visualizeImage()
        {
            cv::imshow("Image", image);
            cv::waitKey(0);
        }

        void visualizePointCloud()
        {
            open3d::visualization::Visualizer vis;
            vis.CreateVisualizerWindow("Point Cloud", 1280, 720);
            vis.AddGeometry(pointcloud);
            vis.Run();
            vis.DestroyVisualizerWindow();
        }

    public:
        std::string image_file, pointcloud_file, frame_pose_file, camera_intrinsics_file, camera_lidar_extrinsics_file;

        void loadInputs()
        {
            if (!image_file.empty()) {
                image = cv::imread(image_file, cv::IMREAD_COLOR);
            }

            if (!pointcloud_file.empty()) {
                cnpy::npz_t pointcloud_ = cnpy::npz_load(pointcloud_file);
                cnpy::NpyArray point_cloud = pointcloud_["points"];
                const float* points_data = point_cloud.data<float>();
                size_t num_points = point_cloud.shape[0];

                //! convert to open3d point_cloud
                auto o3d_point_cloud = std::make_shared<open3d::geometry::PointCloud>();
                o3d_point_cloud->points_.resize(num_points);
                
                for (size_t i = 0; i < num_points; ++i) {
                    o3d_point_cloud->points_[i] = Eigen::Vector3d(points_data[i * 3], points_data[i * 3 + 1], points_data[i * 3 + 2]);
                }
                pointcloud = o3d_point_cloud;
            }

            if (!frame_pose_file.empty()) {
                std::cout << "POSE file: " << frame_pose_file << std::endl;
            }

            if (!camera_intrinsics_file.empty()) {
                camera_intrinsics.load(camera_intrinsics_file);
            }

            if (!camera_lidar_extrinsics_file.empty()) {
                camera_lidar_extrinsics.load(camera_lidar_extrinsics_file);
            }
        }

        void visualizeInputs()
        {
            visualizeImage();
            visualizePointCloud();
            camera_intrinsics.printValues();
            camera_lidar_extrinsics.printValues();
        }
};

} //! end namespace utils

#endif //! UTILS_SRC_REPROJECTION_REPROJECTION_UTILS_H
