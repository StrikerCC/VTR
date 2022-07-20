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
std::shared_ptr<open3d::geometry::VoxelGrid> DilateVoxelGrid(const open3d::geometry::VoxelGrid& voxel_grid_src, int voxel_size_dilating);
std::shared_ptr<open3d::geometry::PointCloud> RemoveOverlappingVoxels(const open3d::geometry::PointCloud& pc_tgt, const open3d::geometry::PointCloud& pc_src, float dilating_voxel=20);
int HasOverlapping(open3d::geometry::VoxelGrid& voxel_grid_tgt, const open3d::geometry::PointCloud& pc_src, open3d::geometry::VoxelGrid& voxel_grid_tgt_overlapping);

int main() {
    std::cout << "Hello, World!" << std::endl;

    bool flag_output_debug = true;
    bool flag_vis_debug = true;

    /// parameter setting
    int voxel_size = 20;

    /// data reading
    std::string data_folder = "./../data/2/";
    int num_camera = 3;
    auto file_paths_camera_parameter = std::make_shared<std::vector<std::string>> ();
    auto file_paths_camera_rgb = std::make_shared<std::vector<std::string>> ();
    auto file_paths_camera_depth = std::make_shared<std::vector<std::string>> ();
    auto file_path_c_arm_motion = std::make_shared<std::string> ("");

    std::cout << "Pwd " << std::filesystem::current_path() << std::endl;
    for (int i_cam = 0; i_cam < num_camera; i_cam++) {
        file_paths_camera_parameter->push_back(data_folder + "/cam_para_" + std::to_string(i_cam) + ".json");
        file_paths_camera_rgb->push_back(data_folder + "/camera" + std::to_string(i_cam) + "_RGB.png");
        file_paths_camera_depth->push_back(data_folder + "/camera" + std::to_string(i_cam) + "_Depth.png");
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
    }

    // c_arm
    std::string c_arm_folder = "./../data/model";
    std::vector<std::string> file_path_c_arm_cad_model = {c_arm_folder + "/c_arm_model/vertical.ply", c_arm_folder + "/c_arm_model/horizontal.ply"};
    auto xyz_min_c_arm_workspace = std::make_shared<Eigen::Vector3d> (800, -1000, 200);
    auto xyz_max_c_arm_workspace = std::make_shared<Eigen::Vector3d> (4500, 1000, 2500);
    auto bbox_c_arm_workspace = std::make_shared<open3d::geometry::AxisAlignedBoundingBox> (*xyz_min_c_arm_workspace, *xyz_max_c_arm_workspace);
//    auto pc_c_arm = std::make_shared<open3d::geometry::TriangleMesh> ();  // mesh from c_arm CAD model
//    open3d::io::ReadTriangleMesh(file_path_c_arm_cad_model.at(0), *pc_c_arm, open3d::io::ReadTriangleMeshOptions());

    auto pose_c_arm_world = Eigen::Matrix4d({{0, 0, 1, -361}, {1, 0, 0, -400}, {0, 1, 0, 150}, {0, 0, 0, 1}});     // transform c_arm to world frame
    auto pc_c_arm = std::make_shared<open3d::geometry::PointCloud> ();  // mesh from c_arm CAD model
    open3d::io::ReadPointCloud(file_path_c_arm_cad_model.at(0), *pc_c_arm);

//    auto voxel_grid_c_arm = open3d::geometry::VoxelGrid::CreateFromTriangleMesh(*pc_c_arm, voxel_size);
    auto voxel_grid_c_arm = open3d::geometry::VoxelGrid::CreateFromPointCloud(*pc_c_arm, voxel_size);
//    auto c_arm_poses_motion_planning = ReadTfFromFile("");
    auto c_arm_poses_motion_planning = std::vector<std::shared_ptr<Eigen::Transform<double, 3, 0x1>>>();
    c_arm_poses_motion_planning.push_back(std::make_shared<Eigen::Transform<double, 3, 0x1>>(pose_c_arm_world));
    c_arm_poses_motion_planning.push_back(std::make_shared<Eigen::Transform<double, 3, 0x1>>(Eigen::Matrix4d({{0, 0, 1, 100}, {1, 0, 0, -400}, {0, 1, 0, 850}, {0, 0, 0, 1}})));
    c_arm_poses_motion_planning.push_back(std::make_shared<Eigen::Transform<double, 3, 0x1>>(Eigen::Matrix4d({{0, 0, 1, 300}, {1, 0, 0, -400}, {0, 1, 0, 150}, {0, 0, 0, 1}})));


    /// visualization
    auto mesh_coord_frame = flag_vis_debug ? std::make_shared<open3d::geometry::TriangleMesh> ()->CreateCoordinateFrame(1000) : nullptr;
    if (bbox_c_arm_workspace && flag_vis_debug) bbox_c_arm_workspace->color_ = Eigen::Vector3d();

//    auto mesh_c_arm_world = std::make_shared<open3d::geometry::PointCloud>(*pc_c_arm);
//    mesh_c_arm_world->Transform(Eigen::Matrix4d({{0, 0, 1, -361}, {1, 0, 0, -400}, {0, 1, 0, 150}, {0, 0, 0, 1}}));     // transform c_arm to world frame
    if (flag_vis_debug) open3d::visualization::DrawGeometries({mesh_coord_frame, bbox_c_arm_workspace, pc_c_arm}, "c_arm from ply file");

    /// statistics
    float cost_avg_build_pc = 0, cost_avg_build_voxel = 0;
    int nLoop = 1;

    /// testing
    for (int i_iter = 0; i_iter < nLoop; i_iter++) {
        /// read point cloud from camera
        auto time_start = std::chrono::high_resolution_clock::now();
        std::vector<std::shared_ptr<open3d::geometry::PointCloud>> pcs;     // point cloud array from triple camera
        auto pc_cam_combined = std::make_shared<open3d::geometry::PointCloud> ();   // point cloud merge from all cameras

        for (int i_cam = 0; i_cam < num_camera; i_cam++) {
            auto pc = open3d::geometry::PointCloud::CreateFromRGBDImage(*rgbds.at(i_cam), cam_models.at(i_cam)->intrinsic, cam_models.at(i_cam)->extrinsic);
            if (flag_output_debug) std::cout << "point cloud remains " << pc->points_.size() << " / " << rgbds.at(i_cam)->color_.width_ * rgbds.at(i_cam)->color_.height_ << " pixels = " << 100.0 * (float) pc->points_.size() / (float) (rgbds.at(i_cam)->color_.width_ * rgbds.at(i_cam)->color_.height_) << " % " << std::endl;
            *pc_cam_combined += *pc;
        }
        float dur_build_point_cloud = (float) std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - time_start).count();
        cost_avg_build_pc += dur_build_point_cloud / (float) nLoop;

        if (flag_output_debug) std::cout << "Point cloud built in " << dur_build_point_cloud << " milliseconds" << std::endl;
        if (flag_vis_debug) open3d::visualization::DrawGeometries({mesh_coord_frame, bbox_c_arm_workspace, pc_cam_combined});

        /// voxel
        time_start = std::chrono::high_resolution_clock::now();
        // build voxel from point cloud
        // TODO: Point Removing
        {
            pc_cam_combined = pc_cam_combined->Crop(*bbox_c_arm_workspace);
            // remove c_arm from camera view point cloud
            auto pc_c_arm_world = *pc_c_arm;
            pc_c_arm_world.Transform(pose_c_arm_world);

            // vis debug
            open3d::visualization::DrawGeometries({pc_cam_combined, std::make_shared<open3d::geometry::PointCloud>(pc_c_arm_world)}, "c_arm pose in real world");

            auto pts_cam_with_c_arm = pc_cam_combined->points_.size();
            pc_cam_combined = RemoveOverlappingVoxels(*pc_cam_combined, pc_c_arm_world);
            std::cout << "remove c_arm: removed " << pts_cam_with_c_arm - pc_cam_combined->points_.size() << " / " << pts_cam_with_c_arm << " = " << float (pts_cam_with_c_arm - pc_cam_combined->points_.size()) / (float) pts_cam_with_c_arm << " points from camera view" << std::endl;

            // vis debug
            open3d::visualization::DrawGeometries({pc_cam_combined, std::make_shared<open3d::geometry::PointCloud>(pc_c_arm_world)}, "c_arm removed in real world");
        }
        auto voxel_grid_room = open3d::geometry::VoxelGrid::CreateFromPointCloud(*pc_cam_combined, voxel_size);

        // vis debug
        open3d::visualization::DrawGeometries({voxel_grid_room}, "c_arm removed from camera view");

        auto dur_build_voxel_grid = (float) std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - time_start).count();
        cost_avg_build_voxel += dur_build_voxel_grid / (float) nLoop;

//        if (flag_vis_debug) open3d::visualization::DrawGeometries({mesh_coord_frame, bbox_c_arm_workspace, voxel_grid_room});
        if (flag_output_debug) std::cout << "Voxel built in " << dur_build_voxel_grid << " milliseconds" << std::endl;

        /// collision detection
        // check overlapping
        for (int i_c_arm_pose = 0; i_c_arm_pose < 10; i_c_arm_pose++) {     // rendering voxel as a_arm motion planned
            auto pc_c_arm_moved_as_plan = *pc_c_arm;
            pc_c_arm_moved_as_plan.Transform(c_arm_poses_motion_planning.at(i_c_arm_pose)->matrix());

            // vis debug
            if (flag_vis_debug) open3d::visualization::DrawGeometries({voxel_grid_room, std::make_shared<open3d::geometry::PointCloud>(pc_c_arm_moved_as_plan)}, "c_arm at pose " + std::to_string(i_c_arm_pose));

            auto voxel_grid_tgt_overlapping = std::make_shared<open3d::geometry::VoxelGrid>();
            int num_of_overlapping = HasOverlapping(*voxel_grid_room, pc_c_arm_moved_as_plan, *voxel_grid_tgt_overlapping);
            // vis debug
            std::cout << "pose " << i_c_arm_pose << " vtr # of collision: " << num_of_overlapping << std::endl;
            open3d::visualization::DrawGeometries({voxel_grid_tgt_overlapping}, "c_arm pose " + std::to_string(i_c_arm_pose) + " overlapping");
            open3d::visualization::DrawGeometries({voxel_grid_tgt_overlapping, std::make_shared<open3d::geometry::PointCloud>(pc_c_arm_moved_as_plan)}, "c_arm pose " + std::to_string(i_c_arm_pose) + " overlapping");
        }

//        if (flag_vis_debug) open3d::visualization::DrawGeometries({voxel_grid_room, voxel_grid_c_arm});
    }

//    std::cout << "collision detected in " << cost_avg_collision_det << " milliseconds" << std::endl;

    return 0;
}

int HasOverlapping(open3d::geometry::VoxelGrid& voxel_grid_tgt, const open3d::geometry::PointCloud& pc_src, open3d::geometry::VoxelGrid& voxel_grid_tgt_overlapping) {
    auto included_src = voxel_grid_tgt.CheckIfIncluded(pc_src.points_);
    {
        auto voxel_grid_tgt_include = std::make_shared<open3d::geometry::VoxelGrid>();
        voxel_grid_tgt_include->Clear();
        voxel_grid_tgt_include->origin_ = voxel_grid_tgt.origin_;
        voxel_grid_tgt_include->voxel_size_ = voxel_grid_tgt.voxel_size_;

        auto voxels_tgt = voxel_grid_tgt.GetVoxels();
        for (int i_pt = 0; i_pt < pc_src.points_.size(); i_pt++) {
            if (included_src.at(i_pt)) voxel_grid_tgt_include->AddVoxel(open3d::geometry::Voxel(voxel_grid_tgt.GetVoxel(pc_src.points_.at(i_pt))));
        }
        std::cout << "num of include " << voxel_grid_tgt_include->voxels_.size() << std::endl;
        open3d::visualization::DrawGeometries({voxel_grid_tgt_include, std::make_shared<open3d::geometry::PointCloud>(pc_src)}, "points include");

    }

    /// find overlapping cluster in target voxel grid
    int id_collision_cluster = 0;
    std::unordered_map<int, Eigen::Vector3d> color_Mapping;
    color_Mapping[0] = Eigen::Vector3d(0.5, 0.5, 0);
    color_Mapping[1] = Eigen::Vector3d(0, 0.5, 0.5);
    color_Mapping[2] = Eigen::Vector3d(0.5, 0, 0.5);
    color_Mapping[3] = Eigen::Vector3d(1, 0, 0);
    color_Mapping[4] = Eigen::Vector3d(0, 1, 0);
    color_Mapping[5] = Eigen::Vector3d(0, 0, 1);
    color_Mapping[6] = Eigen::Vector3d(0.5, 0.5, 1);
    color_Mapping[7] = Eigen::Vector3d(1, 0.5, 0.5);
//    color_Mapping[8] = Eigen::Vector3d(0.5, 1, 0.5);

//    open3d::geometry::VoxelGrid voxel_grid_tgt_overlapping;
    voxel_grid_tgt_overlapping.Clear();
    voxel_grid_tgt_overlapping.voxel_size_ = voxel_grid_tgt.voxel_size_;
    voxel_grid_tgt_overlapping.origin_ = voxel_grid_tgt.origin_;

    std::vector<Eigen::Vector3i> voxel_coords_overlapping;

    for (int i_pt_src = 0; i_pt_src < pc_src.points_.size(); i_pt_src++) {
        if (included_src.at(i_pt_src)) {    // this source point is included in target voxel grid
//            return true;

            // TODO: get cluster
            voxel_coords_overlapping.push_back(voxel_grid_tgt.GetVoxel(pc_src.points_.at(i_pt_src)));

            // deep-first search neighbors
            for (int i_search_level = 0; i_search_level < 10000; i_search_level++) {
                if (voxel_coords_overlapping.empty()) {     // finished one cluster
                    if (i_search_level != 0) id_collision_cluster++;
                    break;
                }

                auto voxel_coord_overlapping = voxel_coords_overlapping.back();
                voxel_coords_overlapping.pop_back();

                // check not same cluster (check if current voxel already marked)
                if (voxel_grid_tgt_overlapping.voxels_.find(voxel_coord_overlapping) == voxel_grid_tgt_overlapping.voxels_.end()
                && voxel_grid_tgt.voxels_.find(voxel_coord_overlapping) != voxel_grid_tgt.voxels_.end()) {
                    // mark cluster by color
                    voxel_grid_tgt_overlapping.AddVoxel(
                            open3d::geometry::Voxel(voxel_coord_overlapping, color_Mapping[int (id_collision_cluster % color_Mapping.size())]));
                    // expand the searching by 8 neighbors
                    for (int x_voxel_local = -1; x_voxel_local < 2; x_voxel_local++) {
                        for (int y_voxel_local = -1; y_voxel_local < 2; y_voxel_local++) {
                            for (int z_voxel_local = -1; z_voxel_local < 2; z_voxel_local++) {
                                if (x_voxel_local == 0 && y_voxel_local == 0 && z_voxel_local == 0) continue;
                                    voxel_coords_overlapping.emplace_back(voxel_coord_overlapping.x() + x_voxel_local,
                                                                          voxel_coord_overlapping.y() + y_voxel_local,
                                                                          voxel_coord_overlapping.z() + z_voxel_local);
                            }
                        }
                    }
                }   // one overlapping cluster
            }       // all overlapping done
        }           // source point overlapped with target point
    }               // check all source points

    /// remap color space
//    for (auto voxel_tgt_overlapping : voxel_grid_tgt_overlapping.voxels_) {
//        voxel_tgt_overlapping.second.color_ /= (double) id_collision_cluster;
//    }

    /// vis debug
//    open3d::visualization::DrawGeometries({std::make_shared<open3d::geometry::VoxelGrid>(voxel_grid_tgt_overlapping)});

    return id_collision_cluster;
}


std::shared_ptr<open3d::geometry::VoxelGrid> DilateVoxelGrid(const open3d::geometry::VoxelGrid& voxel_grid_src, int voxel_size_dilating) {
    auto voxel_grid_dilating = std::make_shared<open3d::geometry::VoxelGrid>(voxel_grid_src);
    open3d::visualization::DrawGeometries({voxel_grid_dilating});

    voxel_grid_dilating->Clear();
    voxel_grid_dilating->voxel_size_ = voxel_grid_src.voxel_size_;
    voxel_grid_dilating->origin_ = voxel_grid_src.origin_;

//    for (const auto &voxel_src : voxel_grid_src.voxels_) {
//        // expand the searching by neighbors
//        for (int x_voxel_local = -voxel_size_dilating; x_voxel_local < voxel_size_dilating + 1; x_voxel_local++) {
//            for (int y_voxel_local = -voxel_size_dilating; y_voxel_local < voxel_size_dilating + 1; y_voxel_local++) {
//                for (int z_voxel_local = -voxel_size_dilating; z_voxel_local < voxel_size_dilating + 1; z_voxel_local++) {
//                    if (x_voxel_local == 0 || y_voxel_local == 0 || z_voxel_local == 0) continue;
//                    if (voxel_grid_dilating->voxels_.find(voxel_src.first) == voxel_grid_dilating->voxels_.end()) {    // this source point is excluded in dilating voxel grid
//                        voxel_grid_dilating->AddVoxel(
//                                open3d::geometry::Voxel(
//                                        {voxel_src.first.x() + x_voxel_local,
//                                         voxel_src.first.y() + y_voxel_local,
//                                         voxel_src.first.z() + z_voxel_local}));
//                    }
//                }
//            }
//        }
//    }

    return voxel_grid_dilating;
}

std::shared_ptr<open3d::geometry::PointCloud> RemoveOverlappingVoxels(const open3d::geometry::PointCloud& pc_tgt, const open3d::geometry::PointCloud& pc_src, float dilating_voxel) {
    float voxel_size = 450;
    // get overlapping index of target point cloud
    auto voxel_grid_src = open3d::geometry::VoxelGrid::CreateFromPointCloud(pc_src, voxel_size);
//    voxel_grid_src = DilateVoxelGrid(*voxel_grid_src, i);

    auto tgt_pts_is_included = voxel_grid_src->CheckIfIncluded(pc_tgt.points_);

    {
        auto pc_tgt_include = std::make_shared<open3d::geometry::PointCloud>();
        auto pc_tgt_not_include = std::make_shared<open3d::geometry::PointCloud>();

        for (int i_pt = 0; i_pt < pc_tgt.points_.size(); i_pt++) {
            if (tgt_pts_is_included.at(i_pt)) {
                pc_tgt_include->points_.push_back(pc_tgt.points_.at(i_pt));
                pc_tgt_include->colors_.push_back({0, 0.5, 0});
            } else {
                pc_tgt_not_include->points_.push_back(pc_tgt.points_.at(i_pt));
                pc_tgt_not_include->colors_.push_back({0, 0, 0.54});
            }
        }
        std::cout << "num of include " << pc_tgt_include->points_.size() << std::endl;
        open3d::visualization::DrawGeometries({std::make_shared<open3d::geometry::PointCloud>(pc_src), pc_tgt_include, pc_tgt_not_include}, "points include");
    }

    // remove overlapping points from target point cloud
    std::vector<Eigen::Vector3d> pts_not_included;
    std::vector<Eigen::Vector3d> colors_not_included;

    for (int i_pt_tgt = 0; i_pt_tgt < pc_tgt.points_.size(); i_pt_tgt++) {
        if (!tgt_pts_is_included[i_pt_tgt]) {
            pts_not_included.push_back(pc_tgt.points_.at(i_pt_tgt));
            colors_not_included.push_back(pc_tgt.colors_.at(i_pt_tgt));
        }
    }
    auto pc_tgt_not_include = std::make_shared<open3d::geometry::PointCloud>(pts_not_included);
    pc_tgt_not_include->colors_ = colors_not_included;


    // vis debug
    open3d::visualization::DrawGeometries({pc_tgt_not_include, voxel_grid_src}, "c_arm pose in real world");


    return pc_tgt_not_include;
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
    double step_t = 100, step_r = 18;
    for (int i_pose = 0; i_pose < 20; i_pose++) {
        auto tf = std::make_shared<Eigen::Transform<double, 3, 0x1>>();
        Eigen::Matrix3d rotation = Eigen::Matrix3d(Eigen::AngleAxisd ((double) i_pose*step_r, Eigen::Vector3d::UnitZ())
                                                   * Eigen::AngleAxisd ((double) i_pose*step_r, Eigen::Vector3d::UnitY())
                                                   * Eigen::AngleAxisd ((double) i_pose*step_r, Eigen::Vector3d::UnitX()));
        tf->translate(Eigen::Vector3d((double) i_pose*step_t, (double) i_pose*step_t, 0));
        tf->rotate(Eigen::Matrix3d(
                Eigen::AngleAxisd ((double) i_pose*step_r, Eigen::Vector3d::UnitZ())
                * Eigen::AngleAxisd ((double) i_pose*step_r, Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd ((double) i_pose*step_r, Eigen::Vector3d::UnitX())));
        tfs.push_back(tf);
    }
    return tfs;
}