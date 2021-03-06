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

//std::vector<Eigen::Transform<double, 3, 0x1>> ReadTfFromFile(const std::string& file_path_motion);
std::vector<Eigen::Matrix4d> ReadTfFromFile(const std::string& file_path_motion);
std::shared_ptr<open3d::geometry::VoxelGrid> DilateVoxelGrid(const open3d::geometry::VoxelGrid& voxel_grid_src, int voxel_size_dilating);
std::shared_ptr<open3d::geometry::PointCloud> RemoveOverlappingVoxels(const open3d::geometry::PointCloud& pc_tgt, const open3d::geometry::PointCloud& pc_src, float dilating_voxel=20);
int HasOverlapping(open3d::geometry::VoxelGrid& voxel_grid_tgt, const open3d::geometry::PointCloud& pc_src, open3d::geometry::VoxelGrid& voxel_grid_tgt_include, open3d::geometry::VoxelGrid& voxel_grid_tgt_component_overlapping);

int main() {
    std::cout << "Hello, World!" << std::endl;

    bool flag_output_debug = true;
    bool flag_vis_debug = true;
    std::string indent = "      ";

    /// parameter setting
    int voxel_size = 10;

    /// data reading
    std::string data_folder = "../../data/2/";
    int num_camera = 3;
    auto file_paths_camera_parameter = std::make_shared<std::vector<std::string>> ();
    auto file_paths_camera_rgb = std::make_shared<std::vector<std::string>> ();
    auto file_paths_camera_depth = std::make_shared<std::vector<std::string>> ();
    auto file_path_c_arm_motion = std::make_shared<std::string> ("");

    std::cout << "$ Pwd: " << std::filesystem::current_path() << std::endl;
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
    std::string c_arm_folder = "../../data/model";
    std::vector<std::string> file_path_c_arm_cad_model = {c_arm_folder + "/c_arm_model/vertical.ply", c_arm_folder + "/c_arm_model/horizontal.ply"};
    auto xyz_min_c_arm_workspace = std::make_shared<Eigen::Vector3d> (800, -1000, 200);
    auto xyz_max_c_arm_workspace = std::make_shared<Eigen::Vector3d> (4500, 1000, 2500);
    auto bbox_c_arm_workspace = std::make_shared<open3d::geometry::AxisAlignedBoundingBox> (*xyz_min_c_arm_workspace, *xyz_max_c_arm_workspace);
//    auto pc_c_arm = std::make_shared<open3d::geometry::TriangleMesh> ();  // mesh from c_arm CAD model
//    open3d::io::ReadTriangleMesh(file_path_c_arm_cad_model.at(0), *pc_c_arm, open3d::io::ReadTriangleMeshOptions());

    auto pose_c_arm_self = Eigen::Matrix4d({{0, 0, 1, -2050}, {1, 0, 0, -500}, {0, 1, 0, -1000}, {0, 0, 0, 1}});     // transform c_arm to world frame
    auto pose_c_arm_world = Eigen::Matrix4d({{1, 0, 0, 1689}, {0, 1, 0, 100}, {0, 0, 1, 1150}, {0, 0, 0, 1}});     // transform c_arm to world frame
    auto pc_c_arm = std::make_shared<open3d::geometry::PointCloud> ();  // mesh from c_arm CAD model
    open3d::io::ReadPointCloud(file_path_c_arm_cad_model.at(0), *pc_c_arm);

//    auto voxel_grid_c_arm = open3d::geometry::VoxelGrid::CreateFromTriangleMesh(*pc_c_arm, voxel_size);
    pc_c_arm->PaintUniformColor({0, 0.5, 0.5});
    auto voxel_grid_c_arm = open3d::geometry::VoxelGrid::CreateFromPointCloud(*pc_c_arm, voxel_size);
//    auto c_arm_poses_motion_planning = std::vector<Eigen::Transform<double, 3, 0x1>>();
    auto c_arm_poses_motion_planning = std::vector<Eigen::Matrix4d>();
    for (const auto &tf_motion_plan : ReadTfFromFile("")) {
        c_arm_poses_motion_planning.emplace_back(pose_c_arm_world * tf_motion_plan * pose_c_arm_self);
    }

    /// visualization
    auto mesh_coord_frame = flag_vis_debug ? std::make_shared<open3d::geometry::TriangleMesh> ()->CreateCoordinateFrame(1000) : nullptr;
    if (bbox_c_arm_workspace && flag_vis_debug) bbox_c_arm_workspace->color_ = Eigen::Vector3d();

    if (flag_vis_debug) open3d::visualization::DrawGeometries({mesh_coord_frame, bbox_c_arm_workspace, pc_c_arm}, "c_arm from ply file");

    /// statistics
    float cost_avg_build_pc = 0, cost_avg_build_voxel = 0;
    int nLoop = 1;

    /// testing
    for (int i_iter = 0; i_iter < nLoop; i_iter++) {
        /// read raw data from camera
        auto time_start = std::chrono::high_resolution_clock::now();
        std::vector<std::shared_ptr<open3d::geometry::PointCloud>> pcs;     // point cloud array from triple camera
        auto pc_cam_combined = std::make_shared<open3d::geometry::PointCloud> ();   // point cloud merge from all cameras

        for (int i_cam = 0; i_cam < num_camera; i_cam++) {
            auto pc = open3d::geometry::PointCloud::CreateFromRGBDImage(*rgbds.at(i_cam), cam_models.at(i_cam)->intrinsic, cam_models.at(i_cam)->extrinsic);
            if (flag_output_debug) std::cout << indent << "read point cloud from rgbd image remains " << pc->points_.size() << " / " << rgbds.at(i_cam)->color_.width_ * rgbds.at(i_cam)->color_.height_ << " pixels = " << 100.0 * (float) pc->points_.size() / (float) (rgbds.at(i_cam)->color_.width_ * rgbds.at(i_cam)->color_.height_) << " % " << std::endl;
            *pc_cam_combined += *pc;
        }
        float dur_data_reading = (float) std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - time_start).count();
        cost_avg_build_pc += dur_data_reading / (float) nLoop;

        if (flag_output_debug) std::cout << "Data read in " << dur_data_reading << " milliseconds" << std::endl;
        if (flag_vis_debug) open3d::visualization::DrawGeometries({mesh_coord_frame, bbox_c_arm_workspace, pc_cam_combined});

        /// preprocess data
        time_start = std::chrono::high_resolution_clock::now();
        // remove c_arm from camera view point cloud
        {
            pc_cam_combined = pc_cam_combined->Crop(*bbox_c_arm_workspace);
            auto pc_c_arm_world = *pc_c_arm;
            pc_c_arm_world.Transform(pose_c_arm_world*pose_c_arm_self);

            // vis debug c_arm real pose in camera view
            if (flag_vis_debug) open3d::visualization::DrawGeometries({pc_cam_combined, std::make_shared<open3d::geometry::PointCloud>(pc_c_arm_world)}, "c_arm pose in real world");

            auto pts_cam_with_c_arm = pc_cam_combined->points_.size();
            pc_cam_combined = RemoveOverlappingVoxels(*pc_cam_combined, pc_c_arm_world);

            // debug c_arm removed from camera view
            if (flag_output_debug) std::cout << indent << "remove c_arm from camera: removed " << pts_cam_with_c_arm - pc_cam_combined->points_.size() << " / " << pts_cam_with_c_arm << " = " << float (pts_cam_with_c_arm - pc_cam_combined->points_.size()) / (float) pts_cam_with_c_arm * 100 << "% points from camera view" << std::endl;
            if (flag_vis_debug) open3d::visualization::DrawGeometries({pc_cam_combined, std::make_shared<open3d::geometry::PointCloud>(pc_c_arm_world)}, "c_arm removed in real world");
        }
        auto voxel_grid_room = open3d::geometry::VoxelGrid::CreateFromPointCloud(*pc_cam_combined, voxel_size);

        auto dur_data_processing = (float) std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - time_start).count();
        cost_avg_build_voxel += dur_data_processing / (float) nLoop;

        if (flag_vis_debug) open3d::visualization::DrawGeometries({voxel_grid_room}, "c_arm removed from camera view");
        if (flag_output_debug) std::cout << "Data precessed (get points in work space, remove c_arm, voxelization) in " << dur_data_processing << " milliseconds" << std::endl;

        /// collision detection
        time_start = std::chrono::high_resolution_clock::now();
        // check overlapping
        for (int i_c_arm_pose = 0; i_c_arm_pose < c_arm_poses_motion_planning.size(); i_c_arm_pose++) {     // rendering voxel as a_arm motion planned
            auto pc_c_arm_moved_as_plan = *pc_c_arm;
            pc_c_arm_moved_as_plan.Transform(c_arm_poses_motion_planning.at(i_c_arm_pose));

            auto voxel_grid_tgt_overlapping = std::make_shared<open3d::geometry::VoxelGrid>();
            auto voxel_grid_tgt_component_overlapping = std::make_shared<open3d::geometry::VoxelGrid>();
            HasOverlapping(*voxel_grid_room, pc_c_arm_moved_as_plan, *voxel_grid_tgt_overlapping, *voxel_grid_tgt_component_overlapping);

            // vis debug
            if (flag_vis_debug) {
                auto voxel_grid_overlapping = open3d::geometry::VoxelGrid();

//                open3d::visualization::DrawGeometries({pc_cam_combined, std::make_shared<open3d::geometry::PointCloud>(pc_c_arm_moved_as_plan)}, "c_arm pose " + std::to_string(i_c_arm_pose) + " overlapping");
                open3d::visualization::DrawGeometries({std::make_shared<open3d::geometry::PointCloud>(pc_c_arm_moved_as_plan), voxel_grid_tgt_overlapping, pc_cam_combined}, "c_arm pose " + std::to_string(i_c_arm_pose) + " overlapping");
//                open3d::visualization::DrawGeometries({voxel_grid_tgt_component_overlapping}, "c_arm pose " + std::to_string(i_c_arm_pose) + " overlapping");
            }
            if (flag_output_debug) std::cout << indent << "# collision " << voxel_grid_tgt_overlapping->voxels_.size() << std::endl;
        }
        float dur_collision = (float) std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - time_start).count();
        if (flag_output_debug) std::cout << "collision detected in " << dur_collision << " milliseconds" << std::endl;
    }
    return 0;
}

int HasOverlapping(open3d::geometry::VoxelGrid& voxel_grid_tgt, const open3d::geometry::PointCloud& pc_src, open3d::geometry::VoxelGrid& voxel_grid_tgt_include, open3d::geometry::VoxelGrid& voxel_grid_tgt_component_overlapping) {
    auto included_src = voxel_grid_tgt.CheckIfIncluded(pc_src.points_);
    {
//        auto voxel_grid_tgt_include = std::make_shared<open3d::geometry::VoxelGrid>();
        voxel_grid_tgt_include.Clear();
        voxel_grid_tgt_include.origin_ = voxel_grid_tgt.origin_;
        voxel_grid_tgt_include.voxel_size_ = voxel_grid_tgt.voxel_size_;

        auto voxels_tgt = voxel_grid_tgt.GetVoxels();
        for (int i_pt = 0; i_pt < pc_src.points_.size(); i_pt++) {
            if (included_src.at(i_pt)) voxel_grid_tgt_include.AddVoxel(open3d::geometry::Voxel(voxel_grid_tgt.GetVoxel(pc_src.points_.at(i_pt)), {1, 0, 0}));
        }
    }

    int id_collision_cluster = 0;                               // cluster index
    std::unordered_map<int, Eigen::Vector3d> color_Mapping;     // cluster color mapping
    color_Mapping[0] = Eigen::Vector3d(0.5, 0.5, 0);
    color_Mapping[1] = Eigen::Vector3d(0, 0.5, 0.5);
    color_Mapping[2] = Eigen::Vector3d(0.5, 0, 0.5);
    color_Mapping[3] = Eigen::Vector3d(0.3, 0, 0);
    color_Mapping[4] = Eigen::Vector3d(0, 0.3, 0);
    color_Mapping[5] = Eigen::Vector3d(0, 0, 0.3);
    color_Mapping[6] = Eigen::Vector3d(0.3, 0.3, 0.5);
    color_Mapping[7] = Eigen::Vector3d(0.5, 0.3, 0.3);
    color_Mapping[8] = Eigen::Vector3d(0.3, 0.5, 0.3);

    voxel_grid_tgt_component_overlapping.Clear();     // overlapping voxel grid
    voxel_grid_tgt_component_overlapping.voxel_size_ = voxel_grid_tgt.voxel_size_;
    voxel_grid_tgt_component_overlapping.origin_ = voxel_grid_tgt.origin_;

    std::vector<Eigen::Vector3i> voxel_coords_overlapping;

    /// find overlapping cluster in target voxel grid
    for (int i_pt_src = 0; i_pt_src < pc_src.points_.size(); i_pt_src++) {
        if (included_src.at(i_pt_src)) {    // this source point is included in target voxel grid
            voxel_coords_overlapping.push_back(voxel_grid_tgt.GetVoxel(pc_src.points_.at(i_pt_src)));

            // deep-first search neighbors
            for (int i_search_level = 0; i_search_level < voxel_grid_tgt.voxels_.size(); i_search_level++) {
                if (voxel_coords_overlapping.empty()) {     // finished one cluster
                    if (i_search_level != 0) id_collision_cluster++;
                    break;
                }

                auto voxel_coord_overlapping = voxel_coords_overlapping.back();
                voxel_coords_overlapping.pop_back();

                // check not same cluster (check if current voxel already marked)
                if (voxel_grid_tgt_component_overlapping.voxels_.find(voxel_coord_overlapping) == voxel_grid_tgt_component_overlapping.voxels_.end()
                    && voxel_grid_tgt.voxels_.find(voxel_coord_overlapping) != voxel_grid_tgt.voxels_.end()) {
                    // mark cluster by color
                    voxel_grid_tgt_component_overlapping.AddVoxel(
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
//    for (auto voxel_tgt_overlapping : voxel_grid_tgt_component_overlapping.voxels_) {
//        voxel_tgt_overlapping.second.color_ /= (double) id_collision_cluster;
//    }

    /// vis debug
//    open3d::visualization::DrawGeometries({std::make_shared<open3d::geometry::VoxelGrid>(voxel_grid_tgt_component_overlapping)});

    return id_collision_cluster;
}


std::shared_ptr<open3d::geometry::VoxelGrid> DilateVoxelGrid(const open3d::geometry::VoxelGrid& voxel_grid_src, int voxel_size_dilating) {
    auto voxel_grid_dilating = std::make_shared<open3d::geometry::VoxelGrid>(voxel_grid_src);
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
                pc_tgt_include->colors_.emplace_back(0, 0.5, 0);
            } else {
                pc_tgt_not_include->points_.push_back(pc_tgt.points_.at(i_pt));
                pc_tgt_not_include->colors_.emplace_back(0, 0, 0.54);
            }
        }
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
//    open3d::visualization::DrawGeometries({pc_tgt_not_include, voxel_grid_src}, "c_arm pose in real world");

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

//std::vector<Eigen::Transform<double, 3, 0x1>> ReadTfFromFile(const std::string& file_path_motion) {
//    float Pi = 3.14159265358979323846;
//    int step_r = 10;
//    std::vector<Eigen::Transform<double, 3, 0x1>> tfs;
//    for (int i_pose = 0; i_pose < step_r; i_pose++) {
//        auto tf = Eigen::Transform<double, 3, 0x1>();
////        Eigen::Matrix3d rotation = Eigen::Matrix3d(Eigen::AngleAxisd ((double) i_pose/step_r * 2*Pi, Eigen::Vector3d::UnitZ())
////                                                   * Eigen::AngleAxisd ((double) 0 * 2*Pi, Eigen::Vector3d::UnitY())
////                                                   * Eigen::AngleAxisd ((double) 0 * 2*Pi, Eigen::Vector3d::UnitX()));
////        tf.translate(Eigen::Vector3d((double) i_pose*step_t, (double) i_pose*step_t, (double) i_pose*step_t));
//        tf.rotate(Eigen::Matrix3d(
//                Eigen::AngleAxisd ((double) i_pose/step_r * 2*Pi, Eigen::Vector3d::UnitZ()))
//                * Eigen::AngleAxisd ((double) 0, Eigen::Vector3d::UnitY())
//                * Eigen::AngleAxisd ((double) 0, Eigen::Vector3d::UnitX())));
//        tfs.push_back(tf);
//    }
//    return tfs;
//}

std::vector<Eigen::Matrix4d> ReadTfFromFile(const std::string& file_path_motion) {
    float Pi = 3.14159265358979323846;
    int step_r = 10;
    std::vector<Eigen::Matrix4d> tfs;
    for (int i_pose = 0; i_pose < step_r; i_pose++) {
        Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();
        tf.block<3, 3>(0, 0) = Eigen::Matrix3d(Eigen::AngleAxisd ((double) 0, Eigen::Vector3d::UnitZ())
                                  * Eigen::AngleAxisd ((double) 0, Eigen::Vector3d::UnitY())
                                  * Eigen::AngleAxisd ((double) i_pose / step_r * 2 * Pi, Eigen::Vector3d::UnitX()));
        tf.block<3, 1>(0, 3) = Eigen::Vector3d({0, 0, 500});
        tfs.push_back(tf);
    }
    return tfs;
}