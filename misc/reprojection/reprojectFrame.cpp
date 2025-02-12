#include "reprojection.h"

int main(int argc, char const *argv[])
{
    if (argc < 5) {
        std::cerr << "Usage: " << argv[0] << " <image_file> <npz_file> <pose> <camera_intrinsics> <camera_lidar_extrinsics>" << std::endl;
        return 1;
    }

    utils::Inputs inputs;
    inputs.image_file = argv[1];
    inputs.pointcloud_file = argv[2];
    inputs.frame_pose_file = argv[3];
    inputs.camera_intrinsics_file = argv[4];
    inputs.camera_lidar_extrinsics_file = argv[5];

    utils::Reprojection reprojection(inputs);

    return 0;
}
