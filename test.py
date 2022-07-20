import os, sys
import time

import numpy as np
import open3d as o3
import json

from slam_lib.camera.cam import PinHoleCamera


def vtr(cams, imgs_rgbd, pc_c_arm):
    vis_flag = True

    # testing
    voxel_size = 10

    tf_c_arm_init = np.eye(4)
    tf_c_arm_init[:3, :3] = np.asarray([[0, 0, 1], [1, 0, 0], [0, 1, 0]])
    tf_c_arm_init[:3, -1] = [-361, -400, 150]

    time_start = time.time()
    pcs, pc_cams = [], o3.geometry.PointCloud()
    for i, (cam, img_rgbd) in enumerate(zip(cams, imgs_rgbd)):
        para_intrinsic = cam.resolution + [cam.camera_matrix[0][0], cam.camera_matrix[1][1],
                                           cam.camera_matrix[0][2], cam.camera_matrix[1][2]]
        intrinsic = o3.camera.PinholeCameraIntrinsic(*para_intrinsic)

        pc = o3.geometry.PointCloud().create_from_rgbd_image(img_rgbd, intrinsic=intrinsic, extrinsic=cam.pose)
        pc_cams += pc

        # if vis_flag:
        # frame = o3.geometry.TriangleMesh().create_coordinate_frame(size=1000)
        # # o3.visualization.draw_geometries([img_rgbd])
        # o3.visualization.draw_geometries([frame, pc])

        # saving camera parameters
        # para_cam = cam.to_dict()
        # f = open(file_paths_cam[i], 'w')
        # json.dump(para_cam, f)

    print('time cost', time.time() - time_start)

    if vis_flag:
        frame = o3.geometry.TriangleMesh().create_coordinate_frame(size=1000)
        o3.visualization.draw_geometries([frame, pc_cams, pc_c_arm])

    # pc to voxel
    voxel_cams = o3.geometry.VoxelGrid().create_from_point_cloud(pc_cams, voxel_size=voxel_size)
    voxel_c_arm = o3.geometry.VoxelGrid().create_from_point_cloud(pc_c_arm, voxel_size=voxel_size)


def get_data():
    vis_flag = True

    file_paths_cam, file_paths_rgb, file_paths_depth = ['./test_img/2/cam_para_0.json',
                                                        './test_img/2/cam_para_1.json',
                                                        './test_img/2/cam_para_2.json'], \
                                                       ['./test_img/2/camera0_RGB.png',
                                                        './test_img/2/camera1_RGB.png',
                                                        './test_img/2/camera2_RGB.png'], \
                                                       ['./test_img/2/camera0_Depth.png',
                                                        './test_img/2/camera1_Depth.png',
                                                        './test_img/2/camera2_Depth.png']
    file_paths_c_arm_model = './c_arm_model/vertical.ply'

    cams = []
    for file_path_cam in file_paths_cam:
        cam = PinHoleCamera()
        file_cam = open(file_path_cam)
        cam_para_dict = json.load(file_cam)
        cam.resolution = cam_para_dict['resolution']
        cam.camera_matrix = cam_para_dict['intrinsic']
        cam.pose = cam_para_dict['extrinsic']
        cam.distortion_coefficient = cam_para_dict['distortion_coefficient']
        cams.append(cam)

    imgs_rgbd = []
    for file_path_rgb, file_path_depth in zip(file_paths_rgb, file_paths_depth):
        img_rgb, img_depth = o3.io.read_image(file_path_rgb), o3.io.read_image(file_path_depth)
        imgs_rgbd.append(
            o3.geometry.RGBDImage().create_from_color_and_depth(img_rgb, img_depth, depth_scale=1, depth_trunc=50000))

    # cams = set_camera_parameter()

    pc_c_arm = o3.io.read_point_cloud(file_paths_c_arm_model)

    return cams, imgs_rgbd, pc_c_arm


def remove_overlapping(voxel_tgt, voxel_src, dilating_voxel_size):
    voxel_size = 20

    # make up ideal data
    tgt_pc = o3.geometry.TriangleMesh().create_sphere(radius=500)
    voxel_tgt = o3.geometry.VoxelGrid().create_from_triangle_mesh(tgt_pc, voxel_size=voxel_size)

    src_pc = o3.geometry.TriangleMesh().create_cone(radius=100, height=1000)
    voxel_src = o3.geometry.VoxelGrid().create_from_triangle_mesh(src_pc, voxel_size=voxel_size)

    # loop over source point
    src_pc = tgt_pc
    is_included = voxel_tgt.check_if_included(src_pc.vertices)
    is_not_included = list(map(lambda x: not x, is_included))

    src_pts_overlapping = np.asarray(src_pc.vertices)[np.asarray(is_included)]
    src_pts_not_overlapping = np.asarray(src_pc.vertices)[np.asarray(is_not_included)]

    tgt_ids_overlapping = [voxel_tgt.get_voxel(src_pt_overlapping) for src_pt_overlapping in src_pts_overlapping]
    tgt_ids_not_overlapping = [voxel_tgt.get_voxel(src_pt_not_overlapping) for src_pt_not_overlapping in src_pts_not_overlapping]

    voxels_tgt = voxel_tgt.get_vexel()
    voxels_tgt[tgt_ids_overlapping]
    voxels_not_overlapping = voxel_tgt.get_voxels()[tgt_ids_not_overlapping]

    # voxel_tgt #= np.asarray([255, 255, 255])
    o3.visualization.draw_geometries([voxel_tgt, voxel_src])
    o3.visualization.draw_geometries([voxels_not_overlapping])
    o3.visualization.draw_geometries([voxels_overlapping])

    o3.geometry.VoxelGrid.get_voxel()
    o3.geometry.TriangleMesh.vertices


def main():
    # cams, imgs_rgbd, pc_c_arm = get_data()
    # vtr(cams, imgs_rgbd, pc_c_arm)

    # TODO: implement voxel overlapping removing
    remove_overlapping(None, None, None)


if __name__ == '__main__':
    main()
