# -*- coding: utf-8 -*-

"""
@author: Cheng Chen
@email: chengc0611@gmail.com
@time: 12/30/21 2:52 PM
"""
import os
import time
import cv2
import numpy as np
import json
from slam_lib.camera.cam import PinHoleCamera, StereoCamera
import slam_lib.feature as feature
import slam_lib.dataset as dataset


def stereo_calibrate(square_size, checkboard_size, left_img_paths, right_img_paths, binocular=None, file_path_2_save=None):
    """

    :param square_size:
    :type square_size:
    :param checkboard_size: (board_width, board_height)
    :type checkboard_size:
    :param left_img_paths:
    :type left_img_paths:
    :param right_img_paths:
    :type right_img_paths:
    :param binocular:stereo amera object
    :type binocular:
    :param file_path_2_save:
    :type file_path_2_save:
    :return:
    :rtype:
    """

    img_size = None
    pts_2d_left, pts_2d_right = [], []

    # corner coord in checkboard frame
    chessboard_corners = feature.make_chessbaord_corners_coord(chessboard_size=checkboard_size,
                                                               square_size=square_size).astype(
        np.float32)
    # chessboard_corners = np.expand_dims(chessboard_corners, axis=-2)
    chessboard_corners = [chessboard_corners] * len(left_img_paths)

    # corner coord in camera frame
    print('looking for all checkboard corners')
    for i, (left_img_name, right_img_name) in enumerate(zip(left_img_paths, right_img_paths)):
        t_start = time.time()
        img_left, img_right = cv2.imread(left_img_name), cv2.imread(right_img_name)
        img_left, img_right = cv2.cvtColor(img_left, cv2.COLOR_BGR2GRAY), cv2.cvtColor(img_right, cv2.COLOR_BGR2GRAY)
        pts_2d_left.append(feature.get_checkboard_corners(img_left, checkboard_size))
        pts_2d_right.append(feature.get_checkboard_corners(img_right, checkboard_size, flag_vis=True))

        if i == 0:
            img_size = (img_left.shape[1], img_left.shape[0])
        print('get ', i, '/'+str(len(left_img_paths)), 'in', time.time() - t_start, 'seconds')
    print('get all checkboard corners')
    # pts_2d_left, pts_2d_right = np.asarray(pts_2d_left), np.asarray(pts_2d_right)

    '''calibrate each camera'''
    ret_left, camera_matrix_left, dist_left, rvecs_left, tvecs_left = cv2.calibrateCamera(chessboard_corners,
                                                                                          pts_2d_left,
                                                                                          imageSize=img_size,
                                                                                          cameraMatrix=np.eye(3),
                                                                                          distCoeffs=np.zeros(5))
    ret_right, camera_matrix_right, dist_right, rvecs_right, tvecs_right = cv2.calibrateCamera(chessboard_corners,
                                                                                               pts_2d_right,
                                                                                               imageSize=img_size,
                                                                                               cameraMatrix=np.eye(3),
                                                                                               distCoeffs=np.zeros(5))

    '''debug'''
    print('Individual camera calibration done')
    print('left camera RMS re-projection error', ret_left)
    print('right camera RMS re-projection error', ret_right)

    '''calibrate binocular'''
    result = cv2.stereoCalibrate(chessboard_corners, pts_2d_left, pts_2d_right, camera_matrix_left, dist_left,
                                 camera_matrix_right, dist_right, imageSize=img_size)
    ret_stereo, camera_matrix_left, dist_left, camera_matrix_right, dist_right, rvec_stereo, tvec_stereo, essential, fundamental = result

    '''build calibration results dict'''
    stereo_calibration_result = {
        'img_size': img_size,
        'camera_matrix_left': camera_matrix_left.tolist(),
        'distortion coefficients_left': dist_left.tolist(),
        'camera_matrix_right': camera_matrix_right.tolist(),
        'distortion coefficients_right': dist_right.tolist(),
        'rotation': rvec_stereo.tolist(),
        'translation': tvec_stereo.tolist(),

        'left_camera_re-projection_error': ret_left,
        'right_camera_re-projection_error': ret_right,
        'stereo_camera_re-projection_error': ret_stereo,
    }

    # results = cv2.stereoRectify(cameraMatrix1=camera_matrix_left, distCoeffs1=dist_left,
    #                            cameraMatrix2=camera_matrix_right,
    #                            distCoeffs2=dist_right, imageSize=img_size, R=cv2.Rodrigues(rvec_stereo)[0],
    #                            T=tvec_stereo)
    # rectify_rotation_left, rectify_rotation_right, rectified_camera_matrix_left, \
    # rectified_camera_matrix_right, Q, roi_left, roi_right = results

    # map_undistort_left, map_rectify_left = cv2.initUndistortRectifyMap(cameraMatrix=camera_matrix_left,
    #                                                                     distCoeffs=dist_left,
    #                                                                     R=rectify_rotation_left,
    #                                                                     newCameraMatrix=rectified_camera_matrix_left,
    #                                                                     size=img_size,
    #                                                                     m1type=cv2.CV_32FC1)
    #
    # map_undistort_right, map_rectify_right = cv2.initUndistortRectifyMap(cameraMatrix=camera_matrix_right,
    #                                                                     distCoeffs=dist_right,
    #                                                                     R=rectify_rotation_right,
    #                                                                     newCameraMatrix=rectified_camera_matrix_right,
    #                                                                     size=img_size,
    #                                                                     m1type=cv2.CV_32FC1)

    '''debug'''
    print('stereo camera RMS re-projection error', ret_stereo)

    '''init camera object parameter'''
    if binocular is not None:
        binocular.set_params(stereo_calibration_result)

    '''save results'''
    if file_path_2_save is not None:
        if not file_path_2_save[-4:] == 'json':
            print(file_path_2_save, 'is not a valid json file path')
        else:
            print('Saving to', file_path_2_save)
            f = open(file_path_2_save, 'w')
            json.dump(stereo_calibration_result, f)
            f.close()
            print('Calibration results saved to', file_path_2_save)

    return True


def main():
    square_size = 0.02423
    checkboard_size = (6, 9)  # (board_width, board_height)

    img_dir = '../data/1/'
    img_left_paths, img_right_paths = dataset.get_left_right_img_path_in_one_folder(img_dir)
    binocular = StereoCamera()
    stereo_calibrate(square_size, checkboard_size, img_left_paths, img_right_paths, binocular=binocular,
                     file_path_2_save='../config/bicam_cal_para.json')

    print('calibration results')
    print(binocular.cam_left.camera_matrix)
    print(binocular.cam_right.camera_matrix)


if __name__ == '__main__':
    main()
