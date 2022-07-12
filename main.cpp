#include <iostream>
#include <string>
#include <chrono>
#include <iostream>
#include <fstream>

#include "open3d/Open3D.h"
#include "nlohmann/json.hpp"

std::shared_ptr<open3d::geometry::PointCloud> CreatePointCloud(const std::string& file_path_cam_setting,
                                                               const std::string& file_path_rgb_img,
                                                               const std::string& file_path_depth_img) {
    open3d::geometry::Image img_rgb, img_depth;
    open3d::camera::PinholeCameraIntrinsic intrinsic;
    Eigen::Matrix4d extrinsic = Eigen::Matrix4d::Identity();

    /*
     * camera parameter
     */
    std::ifstream file_json (file_path_cam_setting);
    nlohmann::json para_cam;
    file_json >> para_cam;
    intrinsic.SetIntrinsics(para_cam["resolution"][0], para_cam["resolution"][1],
                            para_cam["intrinsic"][0][0], para_cam["intrinsic"][1][1],
                            para_cam["intrinsic"][0][2], para_cam["intrinsic"][1][2]);
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            extrinsic(i, j) = (double) para_cam["extrinsic"][i][j];
        }
    }

    /*
     * image reading
     */
    open3d::io::ReadImage(file_path_rgb_img, img_rgb);
    open3d::io::ReadImage(file_path_depth_img, img_depth);
    auto img_rgbd = open3d::geometry::RGBDImage::CreateFromColorAndDepth(img_rgb, img_depth, 1, 12000);
    auto pc = open3d::geometry::PointCloud::CreateFromRGBDImage(*img_rgbd, intrinsic, extrinsic);

    /*
     * vis, log, and cout
     */
    std::cout << "camera intrinsic " << intrinsic.ToString() << std::endl;
    std::cout << "point cloud has " << pc->points_.size() << " from " << img_rgb.width_ * img_rgb.height_ << " pixels" << std::endl;
    return pc;
}

int main() {
    std::cout << "Hello, World!" << std::endl;

    bool vis_debug = false;

    /// hardware setting
    int num_camera = 3;

    auto time_start = std::chrono::high_resolution_clock::now();
    std::cout << "Reading camera parameters and images" << std::endl;
    std::cout << "Pwd " << std::filesystem::current_path() << std::endl;

    // create point cloud from triple camera
    std::vector<std::shared_ptr<open3d::geometry::PointCloud>> pcs;
    open3d::geometry::PointCloud pc_combined;
    for (int i = 0; i < num_camera; i++) {
        auto pc = CreatePointCloud("../data/2/cam_para_" + std::to_string(i) + ".json",
                                     "../data/2/camera" + std::to_string(i) + "_RGB.png",
                                     "../data/2/camera" + std::to_string(i) + "_Depth.png");
        pc_combined += *pc;
        if (vis_debug) open3d::visualization::DrawGeometries({pc});
//        open3d::visualization::DrawGeometries({pcs[i]});
    }

    // merge three point cloud
    auto frame = open3d::geometry::TriangleMesh::CreateCoordinateFrame(1000);
    if (vis_debug) open3d::visualization::DrawGeometries({std::make_shared<open3d::geometry::PointCloud>(pc_combined), frame});

    float dur_build_point_cloud = (float) std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - time_start).count();
    std::cout << "Point cloud built in " << dur_build_point_cloud << " milliseconds" << std::endl;

    time_start = std::chrono::high_resolution_clock::now();
    // build voxel from point cloud
    auto voxel_room = open3d::geometry::VoxelGrid::CreateFromPointCloud(pc_combined, 20);
    if (vis_debug) open3d::visualization::DrawGeometries({voxel_room, frame});
    auto dur_build_voxel_grid = (float) std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - time_start).count();
    std::cout << "Point cloud built in " << dur_build_voxel_grid << " milliseconds" << std::endl;

    return 0;
}
