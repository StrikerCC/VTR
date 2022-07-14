# -*- coding: utf-8 -*-

"""
@author: Cheng Chen
@email: chengc0611@gmail.com
@time: 11/16/21 10:09 AM
"""
import random
# from cam import StereoCamera
import cv2
import numpy as np
import open3d as o3
import transforms3d as tf3


def rt_2_tf(r, t):
    tf = np.eye(4)
    tf[:3, :3] = r
    tf[:3, -1:] = t
    return tf


def scale_pts(scales, pts):
    assert type(pts) == np.ndarray
    assert len(scales) == len(pts.shape)
    if pts.shape[-1] == len(scales):
        return pts * scales
    elif pts.shape[0] == len(scales):
        return (pts.T * scales).T
    else:
        raise AssertionError('expect points shape to be (n, ' + str(len(scales)), ') or (' + str(len(scales)),
                             ', n), but get ' + str(pts.shape) + ' instead')


def transform_pt_3d(tf, pts):
    assert tf.shape == (4, 4)
    if pts.shape[0] == 3:
        return np.matmul(tf[:3, :3], pts) + tf[:3, -1:]
    elif pts.shape[1] == 3:
        return np.matmul(pts, tf[:3, :3].T) + tf[:3, -1:].T
    else:
        raise ValueError('input points shape invalid, expect (n, 3) or (3, n), but get ' + str(pts.shape))


def transform_pt_2d(tf, pts):
    assert tf.shape == (3, 3)
    if pts.shape[0] == 2:
        return np.matmul(tf[:2, :2], pts) + tf[:2, -1:]
    elif pts.shape[1] == 2:
        return np.matmul(pts, tf[:2, :2].T) + tf[:2, -1:].T
    else:
        raise ValueError('input points shape invalid, expect (n, 3) or (3, n), but get ' + str(pts.shape))


def radius(pts_2d, offset=None):
    assert pts_2d.shape[1] == 2
    if offset:
        pts_2d = pts_2d - offset
    return np.linalg.norm(pts_2d, axis=1)


def distort_pt_2d(camera_matrix, distort_coefficient, pts_2d):
    """
    distort points, following opencv conversion
    :param camera_matrix:
    :param distort_coefficient:
    :param pts_2d: (N, 2) camera matrix
    :return:
    """
    assert camera_matrix.shape == (3, 3)
    assert pts_2d.shape[1] == 2
    assert len(pts_2d) > 2

    pts_2d_distorted = np.copy(pts_2d)

    if len(distort_coefficient) == 5:
        '''https://docs.opencv.org/3.4/d4/d94/tutorial_camera_calibration.html'''
        k1, k2, p1, p2, k3 = distort_coefficient
        radius_pts = radius(pts_2d, offset=(camera_matrix[0, -1], camera_matrix[1, -1]))[:, None]
        radius_pts_power = np.power(radius_pts, (2, 4, 6))

        # radial distort
        pts_2d_distorted = pts_2d_distorted + k1 * pts_2d_distorted * radius_pts_power[:, 0:1] + k2 * pts_2d_distorted * \
                           radius_pts_power[:, 1:2] + k3 * pts_2d_distorted * radius_pts_power[:, 2:3]

        # tangential distort
        pts_2d_distorted[:, 0] = pts_2d_distorted[:, 0] + (2 * p1 * pts_2d_distorted[:, 0] * pts_2d_distorted[:, 1] + p2 * (radius_pts_power[:, 0] + 2 * pts_2d_distorted[:, 0]))
        pts_2d_distorted[:, 1] = pts_2d_distorted[:, 1] + (p1 * (radius_pts_power[:, 0] + 2 * pts_2d_distorted[:, 1]) + 2 * p2 * pts_2d_distorted[:, 0] * pts_2d_distorted[:, 1])

    else:
        raise NotImplementedError('distortion for ', len(distort_coefficient), 'coefficient not implemented')

    return pts_2d_distorted


def umeyama_ransac(src, tgt, max_iter=100, confidence=0.9, max_error=0.3):  # compute tf by common pts
    if src.shape[1] != 3:
        src = src.T
    if tgt.shape[1] != 3:
        tgt = tgt.T
    assert src.shape[1] == 3 and tgt.shape[1] == 3, str(src.shape) + ', ' + str(src.shape)

    num_match_pts = 3

    max_inlier_ratio = 0.0
    tf_best = None

    index_list = np.arange(0, len(src))
    for iter in range(max_iter):
        index = np.random.choice(index_list, num_match_pts, replace=False).tolist()  # random pick

        # compute tf
        tf = umeyama(src=src[index].T, tgt=tgt[index].T)

        # compute inlier ratio
        error = np.linalg.norm(tgt - transform_pt_3d(tf, src), axis=-1)  # Manhattan distance
        inlier_ratio = np.count_nonzero(error < max_error) / len(src)

        # break if needed
        if inlier_ratio > confidence:
            return tf

        # update max
        if inlier_ratio >= max_inlier_ratio:
            max_inlier_ratio = inlier_ratio
            tf_best = tf

        print(index)
        print(inlier_ratio)
        print(error)
        print(tf3.euler.mat2euler(tf[:3, :3]))
        print(tf[:3, -1])
        print()

    return tf_best


def umeyama(src, tgt):
    if src.shape[0] != 3:
        src = src.T
    if tgt.shape[0] != 3:
        tgt = tgt.T
    assert src.shape[0] == 3 and tgt.shape[0] == 3, str(src.shape) + ', ' + str(src.shape)

    tf = np.eye(len(src) + 1)
    # rotation
    src_tgt_cov = np.matmul(tgt - np.expand_dims(np.mean(tgt, axis=1), axis=1),
                            np.transpose(src - np.expand_dims(np.mean(src, axis=1), axis=1)))
    u, lam, vt = np.linalg.svd(src_tgt_cov)
    s = np.eye(len(src))
    if np.linalg.det(u) * np.linalg.det(vt) < -0.5:
        s[-1, -1] = -1.0
    tf[:len(src), :len(src)] = np.matmul(np.matmul(u, s), vt)

    # translation
    tf[:-1, -1] = np.mean(tgt, axis=1) - np.mean(np.matmul(tf[:len(src), :len(src)], src), axis=1)
    return tf


def main():
    pts = np.random.random((30, 3))
    # pts = np.vstack([pts, pts+1, pts+2])

    angle_gt = (1, 1, 1)
    tf_gt = np.eye(4)
    tf_gt[:3, :3] = tf3.euler.euler2mat(*angle_gt)
    tf_gt[:3, -1] = (10, 0, 0)

    # print(pts)

    # tf
    pts_tgt = transform_pt_3d(tf_gt, pts)

    # compute tf
    tf = umeyama_ransac(src=pts, tgt=pts_tgt)
    angle = tf3.euler.mat2euler(tf[:3, :3])

    print(pts_tgt)
    print(transform_pt_3d(tf, pts))
    print(np.allclose(pts_tgt, transform_pt_3d(tf, pts)))
    print(angle_gt)
    print(tf_gt[:3, -1])
    print(angle)
    print(tf[:3, -1])

    return


if __name__ == '__main__':
    main()
