#include <iostream>
#include <string>
#include <chrono>
#include <iostream>
#include <fstream>
#include <filesystem>

#include "open3d/Open3D.h"
#include "nlohmann/json.hpp"
#include "Eigen/Eigen"
#include "Eigen/Geometry"

#include "Cam.h"

std::shared_ptr<open3d::geometry::RGBDImage> ReadRGBD(const std::string& file_path_cam_setting,
                                                      const std::string& file_path_rgb_img,
                                                      const std::string& file_path_depth_img);


std::vector<std::shared_ptr<Eigen::Transform<double, 3, 0x1>>> ReadTfFromFile(const std::string& file_path_motion);

int main() {
    std::cout << "Hello, World!" << std::endl;

    bool flag_output_debug = true;
    bool flag_vis_debug = true;

    /// parameter setting
    int voxel_size = 20;

    /// data reading
    int num_camera = 3;
    auto file_paths_camera_parameter = std::make_shared<std::vector<std::string>> ();
    auto file_paths_camera_rgb = std::make_shared<std::vector<std::string>> ();
    auto file_paths_camera_depth = std::make_shared<std::vector<std::string>> ();
    auto file_path_c_arm_motion = std::make_shared<std::string> ("");

    std::cout << "Pwd " << std::filesystem::current_path() << std::endl;
    for (int i_cam = 0; i_cam < num_camera; i_cam++) {
        file_paths_camera_parameter->push_back("../../data/2/cam_para_" + std::to_string(i_cam) + ".json");
        file_paths_camera_rgb->push_back("../../data/2/camera" + std::to_string(i_cam) + "_RGB.png");
        file_paths_camera_depth->push_back("../../data/2/camera" + std::to_string(i_cam) + "_Depth.png");
    }

    /// hardware setting
    // camera
    std::vector<std::shared_ptr<Cam>> cam_models;
    std::vector<std::shared_ptr<open3d::geometry::RGBDImage>> rgbds;
    for (int i_cam = 0; i_cam < num_camera; i_cam++) {
        auto cam = std::make_shared<Cam>();
        cam->SetCamParaFromFile(file_paths_camera_parameter->at(i_cam));
        cam_models.push_back(cam);
        auto rgbd = ReadRGBD(file_paths_camera_parameter->at(i_cam), file_paths_camera_rgb->at(i_cam), file_paths_camera_depth->at(i_cam));
        rgbds.push_back(rgbd);
//        if (flag_output_debug) {
//            std::cout << "camera intrinsic " << cam->intrinsic.ToString() << std::endl;
//            std::cout << "camera extrinsic " << cam->extrinsic << std::endl;
//        }
//        if (flag_vis_debug) open3d::visualization::DrawGeometries({rgbd});
    }

    // c_arm
    std::vector<std::string> file_path_c_arm_cad_model = {"../../data/model/c_arm_model/vertical.ply", "../../data/model/c_arm_model/horizontal.ply"};
    auto xyz_min_c_arm_workspace = std::make_shared<Eigen::Vector3d> (800, -1000, 200);
    auto xyz_max_c_arm_workspace = std::make_shared<Eigen::Vector3d> (4500, 1000, 2500);
    auto bbox_c_arm_workspace = std::make_shared<open3d::geometry::AxisAlignedBoundingBox> (*xyz_min_c_arm_workspace, *xyz_max_c_arm_workspace);
//    auto mesh_c_arm = std::make_shared<open3d::geometry::TriangleMesh> ();  // mesh from c_arm CAD model
//    open3d::io::ReadTriangleMesh(file_path_c_arm_cad_model.at(0), *mesh_c_arm, open3d::io::ReadTriangleMeshOptions());

    auto mesh_c_arm = std::make_shared<open3d::geometry::PointCloud> ();  // mesh from c_arm CAD model
    open3d::io::ReadPointCloud(file_path_c_arm_cad_model.at(0), *mesh_c_arm);

//    auto voxel_c_arm = open3d::geometry::VoxelGrid::CreateFromTriangleMesh(*mesh_c_arm, voxel_size);
    auto voxel_c_arm = open3d::geometry::VoxelGrid::CreateFromPointCloud(*mesh_c_arm, voxel_size);
    auto c_arm_motions = ReadTfFromFile("");

    /// visualization
    auto mesh_frame = flag_vis_debug ? std::make_shared<open3d::geometry::TriangleMesh> ()->CreateCoordinateFrame(1000) : nullptr;
    if (bbox_c_arm_workspace && flag_vis_debug) bbox_c_arm_workspace->color_ = Eigen::Vector3d();

    auto mesh_c_arm_world = std::make_shared<open3d::geometry::PointCloud>(*mesh_c_arm);
    mesh_c_arm_world->Transform(Eigen::Matrix4d({{0, 0, 1, -361}, {1, 0, 0, -400}, {0, 1, 0, 150}, {0, 0, 0, 1}}));     // transform c_arm to world frame
    if (flag_vis_debug) open3d::visualization::DrawGeometries({mesh_frame, bbox_c_arm_workspace, mesh_c_arm, mesh_c_arm_world}, "c_arm from ply file");

    /// statistics
    float cost_avg_build_pc, cost_avg_build_voxel;
    int nLoop = 100;

    /// testing
    for (int i_iter = 0; i_iter < nLoop; i_iter++) {
        /// read point cloud from camera
        auto time_start = std::chrono::high_resolution_clock::now();
        std::vector<std::shared_ptr<open3d::geometry::PointCloud>> pcs;     // point cloud array from triple camera
        auto pc_combined = std::make_shared<open3d::geometry::PointCloud> ();   // point cloud merge from all cameras

        for (int i_cam = 0; i_cam < num_camera; i_cam++) {
            auto pc = open3d::geometry::PointCloud::CreateFromRGBDImage(*rgbds.at(i_cam), cam_models.at(i_cam)->intrinsic, cam_models.at(i_cam)->extrinsic);
            if (flag_output_debug) std::cout << "point cloud remains " << pc->points_.size() << " / " << rgbds.at(i_cam)->color_.width_ * rgbds.at(i_cam)->color_.height_ << " pixels = " << 100.0 * (float) pc->points_.size() / (float) (rgbds.at(i_cam)->color_.width_ * rgbds.at(i_cam)->color_.height_) << " % " << std::endl;
            *pc_combined += *pc;
        }
        float dur_build_point_cloud = (float) std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - time_start).count();
        cost_avg_build_pc += dur_build_point_cloud / (float) nLoop;

        if (flag_output_debug) std::cout << "Point cloud built in " << dur_build_point_cloud << " milliseconds" << std::endl;
        if (flag_vis_debug) open3d::visualization::DrawGeometries({mesh_frame, bbox_c_arm_workspace, pc_combined});

        /// voxel
        time_start = std::chrono::high_resolution_clock::now();
        // build voxel from point cloud
        // TODO: CreateFromPointCloudWithinBounds doesn't filter point side bound
//        auto voxel_room = open3d::geometry::VoxelGrid::CreateFromPointCloudWithinBounds(*pc_combined, voxel_size, *xyz_min_c_arm_workspace, *xyz_max_c_arm_workspace);
        // TODO: Point Crop has no inplace change function
        auto voxel_room = open3d::geometry::VoxelGrid::CreateFromPointCloud(*pc_combined->Crop(*bbox_c_arm_workspace), voxel_size);
        auto dur_build_voxel_grid = (float) std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - time_start).count();
        cost_avg_build_voxel += dur_build_voxel_grid / (float) nLoop;

        if (flag_vis_debug) open3d::visualization::DrawGeometries({mesh_frame, bbox_c_arm_workspace, voxel_room});
        if (flag_output_debug) std::cout << "Voxel built in " << dur_build_voxel_grid << " milliseconds" << std::endl;

        /// collision detection
//        auto voxel_c_arm_trajectory = std::make_shared<open3d::geometry::VoxelGrid> ();
        for (int i_c_arm_pose = 0; i_c_arm_pose < 10; i_c_arm_pose++) {     // rendering voxel as a_arm motion planned
            voxel_c_arm->Transform(c_arm_motions.at(i_c_arm_pose)->matrix());
//            *voxel_c_arm_trajectory += *voxel_c_arm;
        }

        if (flag_vis_debug) open3d::visualization::DrawGeometries({mesh_frame, bbox_c_arm_workspace, voxel_room, voxel_c_arm});

    }

    std::cout << "Point cloud built in " << cost_avg_build_pc << " milliseconds" << std::endl;
    std::cout << "Voxel built in " << cost_avg_build_voxel << " milliseconds" << std::endl;

    return 0;
}

std::shared_ptr<open3d::geometry::RGBDImage> ReadRGBD(const std::string& file_path_cam_setting,
                                                      const std::string& file_path_rgb_img,
                                                      const std::string& file_path_depth_img) {
    open3d::geometry::Image img_rgb, img_depth;

    // image reading
    open3d::io::ReadImage(file_path_rgb_img, img_rgb);
    open3d::io::ReadImage(file_path_depth_img, img_depth);
    auto img_rgbd = open3d::geometry::RGBDImage::CreateFromColorAndDepth(img_rgb, img_depth, 1, 12000);
    return img_rgbd;
}

std::vector<std::shared_ptr<Eigen::Transform<double, 3, 0x1>>> ReadTfFromFile(const std::string& file_path_motion) {
    std::vector<std::shared_ptr<Eigen::Transform<double, 3, 0x1>>> tfs;
    float step_t = 100, step_r = 18;
    for (int i_pose = 0; i_pose < 20; i_pose++) {
        auto tf = std::make_shared<Eigen::Transform<float, 3, 0x1>>();
        Eigen::Matrix3f rotation;
        tf->translate(Eigen::Vector3f((float) i_pose*step_t, (float) i_pose*step_t, 0));
        rotation = Eigen::AngleAxisf ((float) i_pose*step_r, Eigen::Vector3f::UnitZ())
                * Eigen::AngleAxisf ((float) i_pose*step_r, Eigen::Vector3f::UnitY())
                * Eigen::AngleAxisf ((float) i_pose*step_r, Eigen::Vector3f::UnitX());
        tf->rotate(rotation);
    }
    return tfs;
}