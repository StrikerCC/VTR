import os, sys
import time

import numpy as np
import open3d as o3
import json

from slam_lib.camera.cam import PinHoleCamera


# def set_camera_parameter():
#     cam_0, cam_1, cam_2 = PinHoleCamera(), PinHoleCamera(), PinHoleCamera()
#     cam_0.resolution, cam_1.resolution, cam_2.resolution = [640, 480],  [640, 480],  [640, 480]
#     cam_0.camera_matrix, cam_1.camera_matrix, cam_2.camera_matrix = [[585.65, 0, 336.2],
#                                                                      [0, 585.65, 241.6],
#                                                                      [0, 0, 1]], \
#                                                                     [[585.65, 0, 336.2],
#                                                                      [0, 585.65, 241.6],
#                                                                      [0, 0, 1]],  \
#                                                                     [[585.65, 0, 336.2],
#                                                                      [0, 585.65, 241.6],
#                                                                      [0, 0, 1]]
#     cam_0.pose, cam_1.pose, cam_2.pose = [[0.98825103, 0.0572171137, -0.141725525, -3065.48853],
#                                           [-0.0449036434, -0.777667, -0.6270707, 607.3281],
#                                           [-0.146094441, 0.6260673, -0.765960932, 4155.07373],
#                                           [0, 0, 0, 1]], \
#                                          [[-0.983930767, 0.000168152881, 0.1785504, 2934.36841],
#                                           [-0.105896972, 0.8045843, -0.584320068, 617.411],
#                                           [-0.1437571, -0.5938384, -0.7916374, 4215.391],
#                                           [0, 0, 0, 1]], \
#                                          [[-0.889865041, -0.017826112, 0.455875516, 2353.53662],
#                                           [0.03903659, 0.992596745, 0.115012616, -204.76236],
#                                           [-0.454550743, 0.120141529, -0.882581234, 4662.15625],
#                                           [0, 0, 0, 1]]
#     return cam_0, cam_1, cam_2
#
# def get_camera_parameter():
#     # ---uvision camera intrincs---
#     focalLength = 585.65
#     centerX = 336.2
#     centerY = 241.6
#     # ---camera 0 extrinsic---
#     camera_0_extrinsic = {  # rotation matrix
#         'R': [[0.98825103, 0.0572171137, -0.141725525],
#             [-0.0449036434, -0.777667, -0.6270707],
#             [-0.146094441, 0.6260673, -0.765960932]],
#         # translation vector
#         'T': [-3065.48853, 607.3281, 4155.07373]}
#
#     # ---camera 1 extrinsic---
#     camera_1_extrinsic = {  # rotation matrix
#         'R': [[-0.983930767, 0.000168152881, 0.1785504],
#             [-0.105896972, 0.8045843, -0.584320068],
#             [-0.1437571, -0.5938384, -0.7916374]],
#         # translation vector
#         'T': [2934.36841, 617.411, 4215.391]}
#
#     # ---camera 2 extrinsic---
#     camera_2_extrinsic = {  # rotation matrix
#         'R': [[-0.889865041, -0.017826112, 0.455875516],
#             [0.03903659, 0.992596745, 0.115012616],
#             [-0.454550743, 0.120141529, -0.882581234]],
#         # translation vector
#         'T': [2353.53662, -204.76236, 4662.15625]}
#     camera_extrinsic = {'0': camera_0_extrinsic, '1': camera_1_extrinsic, '2': camera_2_extrinsic}
#     camera_intrinsic=np.array([focalLength,centerX,centerY])
#     return camera_extrinsic,camera_intrinsic


def get_data():

    vis_flag = True
    time_start = time.time()

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
        imgs_rgbd.append(o3.geometry.RGBDImage().create_from_color_and_depth(img_rgb, img_depth, depth_scale=1, depth_trunc=50000))

    # cams = set_camera_parameter()

    pcs, pc_combined = [], o3.geometry.PointCloud()
    for i, (cam, img_rgbd) in enumerate(zip(cams, imgs_rgbd)):
        para_intrinsic = cam.resolution + [cam.camera_matrix[0][0], cam.camera_matrix[1][1],
                                           cam.camera_matrix[0][2], cam.camera_matrix[1][2]]
        intrinsic = o3.camera.PinholeCameraIntrinsic(*para_intrinsic)

        pc = o3.geometry.PointCloud().create_from_rgbd_image(img_rgbd, intrinsic=intrinsic, extrinsic=cam.pose)
        pc_combined += pc

        # if vis_flag:
        # frame = o3.geometry.TriangleMesh().create_coordinate_frame(size=1000)
        # # o3.visualization.draw_geometries([img_rgbd])
        # o3.visualization.draw_geometries([frame, pc])

        # saving camera parameters
        # para_cam = cam.to_dict()
        # f = open(file_paths_cam[i], 'w')
        # json.dump(para_cam, f)

    print('time cost', time.time() - time_start)

    pc_c_arm = o3.io.read_point_cloud(file_paths_c_arm_model)
    tf = np.eye(4)
    tf[:3, :3] = np.asarray([[0, 0, 1], [1, 0, 0], [0, 1, 0]])
    tf[:3, -1] = [-361, -400, 150]
    pc_c_arm.transform(tf)

    frame = o3.geometry.TriangleMesh().create_coordinate_frame(size=1000)

    if vis_flag:
        o3.visualization.draw_geometries([frame, pc_combined, pc_c_arm])

    ## pc to voxel
    o3.geometry.VoxelGrid.create_from_point_cloud(pc_c_arm)

def main():
    get_data()


if __name__ == '__main__':
    main()
