# -*- coding: utf-8 -*-

"""
@author: Cheng Chen
@email: chengc0611@gmail.com
@time: 12/30/21 2:56 PM
"""
import json
import os
import time

import cv2
import numpy as np

import slam_lib.mapping as mapping


class PinHoleCamera:
    def __init__(self):
        self.resolution = None
        self.camera_matrix = None
        self.pose = None
        self.distortion_coefficient = None

    def proj_pts_3d_camera_frame_2_img_frame(self, pts_3d):
        """
        project 3d points in camera frame to pixel frame
        :param pts_3d: (n, 3)
        :type pts_3d:
        :return:
        :rtype:
        """
        if pts_3d.shape[0] != 3:
            pts_3d = pts_3d.T  # (3, N)
        assert len(pts_3d.shape) == 2 and pts_3d.shape[0] == 3

        pts_2d, *_ = cv2.projectPoints(pts_3d, rvec=np.eye(3), tvec=np.zeros(3), cameraMatrix=self.camera_matrix,
                                       distCoeffs=self.distortion_coefficient)
        return pts_2d[:, 0, :]

    def proj_pts_2d_img_frame_2_new_frame(self, pts_2d, rotation=None, camera_matrix=None):
        """
        :param camera_matrix: 
        :param rotation: 
        :param pts_2d: x, y raw coord in opencv format
        :return:
        """
        if pts_2d.shape[1] == 2:
            pts_2d = pts_2d.T

        pts_2d_rectified = cv2.undistortPoints(src=pts_2d, cameraMatrix=self.camera_matrix,
                                               distCoeffs=self.distortion_coefficient, R=rotation, P=camera_matrix)
        return pts_2d_rectified[:, 0, :]

    def to_dict(self):
        """
        convert camera parameters to json data
        :return: camera parameters
        :rtype: json
        """
        return {
            'resolution': self.resolution,
            'intrinsic': self.camera_matrix,
            'extrinsic': self.pose,
            'distortion_coefficient': self.distortion_coefficient,
        }


class StereoCamera:
    t = None
    r = None
    img_size = None
    rotation_rectify_left, camera_matrix_rectify_left = None, None
    rotation_rectify_right, camera_matrix_rectify_right = None, None

    def __init__(self, para_file_path=None):
        self.cam_left, self.cam_right = PinHoleCamera(), PinHoleCamera()
        if para_file_path is not None:
            if os.path.isfile(str(para_file_path)):
                self.read_cal_param_file_and_set_params(para_file_path)
            else:
                print('calibration file doesn\'t exist at', para_file_path)

    def read_cal_param_file_and_set_params(self, para_file_path):
        if not os.path.isfile(str(para_file_path)):
            print('calibration file read fail at', para_file_path)
            return False
        if para_file_path[-3:] == 'xml':
            f = cv2.cv2.FileStorage(para_file_path, cv2.cv2.FILE_STORAGE_READ)
            self.cam_left.camera_matrix, self.cam_right.camera_matrix = \
                f.getNode('leftCameraMatrix').mat(), f.getNode('rightCameraMatrix').mat()
            self.cam_left.distortion_coefficient, self.cam_right.distortion_coefficient = \
                f.getNode('leftDistCoeffs').mat(), f.getNode('rightDistCoeffs').mat()
            self.r, self.t = f.getNode('R').mat(), f.getNode('T').mat()
            self.e, self.f = f.getNode('E').mat(), f.getNode('F').mat()
            self.rotation_rectify_left, self.camera_matrix_rectify_left, \
            self.rotation_rectify_right, self.camera_matrix_rectify_right, \
            self.Q = f.getNode('R1').mat(), f.getNode('P1').mat(), \
                     f.getNode('R2').mat(), f.getNode('P2').mat(), \
                     f.getNode('Q').mat()
            return True
        elif para_file_path[-4:] == 'json':
            print('read camera parameter from ', para_file_path)
            f = open(para_file_path, 'r')
            params = json.load(f)
            self.set_params(params)
            return True

    def set_params(self, params):
        print('setting camera parameters')
        assert isinstance(params, dict), type(params)
        self.img_size = params['img_size']
        self.cam_left.camera_matrix = np.asarray(params['camera_matrix_left'])
        self.cam_right.camera_matrix = np.asarray(params['camera_matrix_right'])
        self.cam_left.distortion_coefficient = np.asarray(params['distortion coefficients_left'])
        self.cam_right.distortion_coefficient = np.asarray(params['distortion coefficients_right'])
        self.r = np.asarray(params['rotation'])
        self.t = np.asarray(params['translation'])

        time_start = time.time()
        print('init undistortion and rectify mapping')
        result = cv2.stereoRectify(cameraMatrix1=self.cam_left.camera_matrix,
                                   distCoeffs1=self.cam_left.distortion_coefficient,
                                   cameraMatrix2=self.cam_right.camera_matrix,
                                   distCoeffs2=self.cam_right.distortion_coefficient, imageSize=self.img_size,
                                   R=cv2.Rodrigues(self.r)[0], T=self.t)

        rotation_rectify_left, rotation_rectify_right, camera_matrix_rectify_left, camera_matrix_rectified_right, Q, \
        roi_left, roi_right = result

        # map_undistort, map_rectify = cv2.initUndistortRectifyMap(
        #     cameraMatrix=self.cam_left.camera_matrix,
        #     distCoeffs=self.cam_left.distortion_coefficient,
        #     R=rotation_rectify_left,
        #     newCameraMatrix=camera_matrix_rectify_left,
        #     size=self.img_size,
        #     m1type=cv2.CV_32FC1)
        self.rotation_rectify_left = rotation_rectify_left
        self.camera_matrix_rectify_left = camera_matrix_rectify_left
        # self.cam_left.map_rectify = np.asarray([map_undistort, map_rectify]).transpose((1, 2, 0))

        # map_undistort, map_rectify = cv2.initUndistortRectifyMap(
        #     cameraMatrix=self.cam_right.camera_matrix,
        #     distCoeffs=self.cam_right.distortion_coefficient,
        #     R=rotation_rectify_right,
        #     newCameraMatrix=camera_matrix_rectified_right,
        #     size=self.img_size,
        #     m1type=cv2.CV_32FC1)
        self.rotation_rectify_right = rotation_rectify_right
        self.camera_matrix_rectify_right = camera_matrix_rectified_right
        # self.cam_right.map_rectify = np.asarray([map_undistort, map_rectify]).transpose((1, 2, 0))
        self.Q = Q

        print('left_camera_reprojection_error', params["left_camera_re-projection_error"])
        print("right_camera_reprojection_error", params['right_camera_re-projection_error'])
        print("stereo_camera_reprojection_error", params["stereo_camera_re-projection_error"])
        print('undistortion and rectify mapping set in ', time.time() - time_start, 'second')
        return True

    def correspondence_rectified_to_3d_in_left_rectified(self, pts_2d_left, pts_2d_right):
        pts_2d_left, pts_2d_right = pts_2d_left.reshape(-1, 2), pts_2d_right.reshape(-1, 2)
        disparity = pts_2d_left[:, 0] - pts_2d_right[:, 0]
        _Q = self.Q
        pt_homo = np.matmul(_Q, np.vstack([pts_2d_left[:, 0], pts_2d_left[:, 1], disparity, np.ones(len(disparity))]))
        pts = pt_homo[:-1, :]
        pts /= pt_homo[3, :]
        return pts.T

    def correspondence_to_3d_in_left(self, pts_2d_left, pts_2d_right):
        """

        :param pts_2d_left:
        :param pts_2d_right:
        :return:
        """
        '''rectify points in image frame'''
        pts_2d_left, pts_2d_right = pts_2d_left.reshape(-1, 2), pts_2d_right.reshape(-1, 2)
        pts_2d_left = self.cam_left.proj_pts_2d_img_frame_2_new_frame(pts_2d_left, self.rotation_rectify_left,
                                                                      self.camera_matrix_rectify_left)
        pts_2d_right = self.cam_right.proj_pts_2d_img_frame_2_new_frame(pts_2d_right, self.rotation_rectify_right,
                                                                        self.camera_matrix_rectify_right)

        '''compute points 3d coord in rectified camera frame'''
        pts_3d_in_left_rectify = self.correspondence_rectified_to_3d_in_left_rectified(pts_2d_left, pts_2d_right)

        '''map points 3d from rectified camera frame to original camera frame'''
        tf_left_2_left_rectify = mapping.rt_2_tf(self.rotation_rectify_left, np.zeros((3, 1)))
        pts_3d_in_left = mapping.transform_pt_3d(np.linalg.inv(tf_left_2_left_rectify), pts_3d_in_left_rectify)
        return pts_3d_in_left

    def proj_pts_3d_2_rectified_img_frames(self, pts_3d):
        assert pts_3d.shape[1] == 3
        pts_2d_left_rectified_cam, *_ = cv2.projectPoints(
            pts_3d, rvec=cv2.Rodrigues(self.rotation_rectify_left)[0], tvec=np.zeros(3),
            cameraMatrix=self.camera_matrix_rectify_left, distCoeffs=None)

        pts_2d_right_rectified_cam, *_ = cv2.projectPoints(
            pts_3d, rvec=cv2.Rodrigues(self.rotation_rectify_right)[0], tvec=np.zeros(3),
            cameraMatrix=self.camera_matrix_rectify_right, distCoeffs=None)

        return pts_2d_left_rectified_cam[:, 0, :], pts_2d_right_rectified_cam[:, 0, :]


def main():
    bcmaera = StereoCamera('/home/cheng/proj/3d/BiCameraSDKv2.0/test_data/saved_parameters.xml')
    pt_left, pt_right = np.array([220, 220]), np.array([100, 100])
    world = bcmaera.correspondence_rectified_to_3d_in_left_rectified(pt_left, pt_right)
    print(world)


if __name__ == '__main__':
    main()
