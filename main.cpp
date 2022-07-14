#include <iostream>
#include <string>
#include <chrono>
#include <iostream>
#include <fstream>
#include <filesystem>

#include "open3d/Open3D.h"
#include "nlohmann/json.hpp"
#include "Eigen/Eigen"

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
    // std::cout << "camera intrinsic " << intrinsic.ToString() << std::endl;
    // std::cout << "point cloud has " << pc->points_.size() << " from " << img_rgb.width_ * img_rgb.height_ << " pixels" << std::endl;
    return pc;
}

int main() {
    std::cout << "Hello, World!" << std::endl;
    /*
    * hardware setting
    */
    // camera
    int num_camera = 3;

    // c_arm
    auto xyz_min = std::make_shared<Eigen::Vector3d> (800, -1000, 200);
    auto xyz_max = std::make_shared<Eigen::Vector3d> (4500, 1000, 2500);
    auto mesh_bound = std::make_shared<open3d::geometry::AxisAlignedBoundingBox> (*xyz_min, *xyz_max);
    std::string file_path_c_arm_cad_model = "";
//    auto xyz_min = std::make_shared<Eigen::Vector3d> (0, 0, 0);
//    auto xyz_max = std::make_shared<Eigen::Vector3d> (1000, 1000, 1000);

    int voxel_size = 20;

    // testing setting
    float cost_avg_build_pc = 0.0, cost_avg_build_voxel = 0.0;
    int nLoop = 100;

    // debug output
    bool flag_output_debug = false;

    // debug visualization
    bool flag_vis_debug = false;
    auto mesh_frame = flag_vis_debug ? std::make_shared<open3d::geometry::TriangleMesh> ()->CreateCoordinateFrame(1000) : nullptr;
    if (mesh_bound && flag_vis_debug) mesh_bound->color_ = Eigen::Vector3d();

    /*
     * testing loop
     */
    for (int i_iter = 0; i_iter < nLoop; i_iter++) {
        
        /*
         * point cloud and mesh
         */
        if (flag_output_debug) {
            std::cout << "Reading camera parameters and images" << std::endl;
            std::cout << "Pwd " << std::filesystem::current_path() << std::endl;
        }

        // point cloud from camera
        auto time_start = std::chrono::high_resolution_clock::now();
        std::vector<std::shared_ptr<open3d::geometry::PointCloud>> pcs;     // point cloud array from triple camera
        auto pc_combined = std::make_shared<open3d::geometry::PointCloud> ();   // point cloud merge from all cameras

        for (int i_cam = 0; i_cam < num_camera; i_cam++) {
            auto pc = CreatePointCloud("../../data/2/cam_para_" + std::to_string(i_cam) + ".json",
                                       "../../data/2/camera" + std::to_string(i_cam) + "_RGB.png",
                                       "../../data/2/camera" + std::to_string(i_cam) + "_Depth.png");
            *pc_combined += *pc;
            if (flag_vis_debug) open3d::visualization::DrawGeometries({mesh_frame, mesh_bound, pc});
        }
        if (flag_vis_debug) open3d::visualization::DrawGeometries({mesh_frame, mesh_bound, pc_combined});

        float dur_build_point_cloud = (float) std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - time_start).count();
        if (flag_output_debug) std::cout << "Point cloud built in " << dur_build_point_cloud << " milliseconds" << std::endl;
        cost_avg_build_pc += dur_build_point_cloud / (float) nLoop;

        // mesh from c_arm CAD model
        auto mesh_c_arm = std::make_shared<open3d::geometry::TriangleMesh> ();
        open3d::io::ReadTriangleMeshFromPLY(file_path_c_arm_cad_model, *mesh_c_arm);

        /*
         * voxel
         */
        time_start = std::chrono::high_resolution_clock::now();

        /// build voxel from point cloud
        /// TODO: CreateFromPointCloudWithinBounds doesn't filter point side bound
        auto voxel_room = open3d::geometry::VoxelGrid::CreateFromPointCloudWithinBounds(*pc_combined->Crop(*mesh_bound), voxel_size, *xyz_min, *xyz_max);
//        auto voxel_room = open3d::geometry::VoxelGrid::CreateFromPointCloudWithinBounds(*pc_combined, voxel_size, *xyz_min, *xyz_max);

        if (flag_vis_debug) open3d::visualization::DrawGeometries({mesh_frame, mesh_bound, voxel_room});

        auto dur_build_voxel_grid = (float) std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - time_start).count();
        if (flag_output_debug) std::cout << "Voxel built in " << dur_build_voxel_grid << " milliseconds" << std::endl;
        cost_avg_build_voxel += dur_build_voxel_grid / (float) nLoop;

        auto voxel_c_arm_model = open3d::geometry::VoxelGrid::CreateFromTriangleMesh(*mesh_c_arm, voxel_size);

        /// collision detection

    }

    std::cout << "Point cloud built in " << cost_avg_build_pc << " milliseconds" << std::endl;
    std::cout << "Voxel built in " << cost_avg_build_voxel << " milliseconds" << std::endl;
    return 0;
}
