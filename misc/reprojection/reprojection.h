#ifndef UTILS_SRC_REPROJECTION_REPROJECTION_H
#define UTILS_SRC_REPROJECTION_REPROJECTION_H

#include "reprojectionUtils.h"
#define DEBUG_PRINT(x) std::cout << x << std::endl

namespace utils
{

class Reprojection
{
    private:
        Inputs reprojectionInputs_;

        void loadParams();

        cv::Mat projectLidarToImage(const cv::Mat& input_image, 
                                    const std::shared_ptr<open3d::geometry::PointCloud>& input_pointcloud,
                                    const Eigen::Isometry3f& camera_T_lidar);

        cv::Mat convertProjection(const std::shared_ptr<open3d::geometry::PointCloud>& input_pointcloud,
                                  std::vector<std::pair<bool, cv::Point2i>>& inverse_lookup_table,
                                  const cv::Size& image_size,
                                  const Eigen::Isometry3f& camera_T_lidar);

    public:
        Reprojection(const Inputs& inputs_);

        ~Reprojection() {}
};

} //! end namespace utils

#endif //! UTILS_SRC_REPROJECTION_REPROJECTION_H
