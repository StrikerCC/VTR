# -*- coding: utf-8 -*-

"""
@author: Cheng Chen
@email: chengc0611@gmail.com
@time: 12/9/21 5:30 PM
"""
import copy
import numpy as np
import math

import cv2
import transforms3d as t3d
import scipy.spatial.kdtree
import slam_lib.vis as vis


def rot2d(angle):
    return np.asarray([[math.cos(angle), -math.sin(angle)],
                       [math.sin(angle), math.cos(angle)]])


def affine2d():
    return np.array([[1.0, -0.1], [-0.1, 1.0]])


def rt2d(angle, x=0, y=0):
    rot = rot2d(angle)
    translation = np.asarray([[x], [y]])
    return np.concatenate([rot, translation], axis=1)


def affinert2d(angle, x=0, y=0):
    rot = rot2d(angle)
    translation = np.asarray([[x], [y]])
    affine = affine2d()
    affiner = np.matmul(rot, affine)
    affinert = np.concatenate([affiner, translation], axis=1)
    return affinert


def tsfm(img, start_points, end_points, angle=math.pi / 120, flag_vis=False):
    img_r, start_points_r, end_points_r = copy.deepcopy((img, start_points, end_points))
    start_points_r, end_points_r = np.concatenate([start_points_r, np.ones((len(start_points_r), 1))], axis=1), \
                                   np.concatenate([end_points_r, np.ones((len(end_points_r), 1))], axis=1)
    affine = affinert2d(angle, x=50, y=20)
    img_r = cv2.warpAffine(img_r, M=affine, dsize=(img_r.shape[1], img_r.shape[0]))
    start_points_r = np.dot(affine, start_points_r.T).T
    end_points_r = np.dot(affine, end_points_r.T).T
    if flag_vis:
        for start_point_r in start_points_r:
            if np.alltrue(start_point_r > 0) and np.alltrue(start_point_r < np.asarray(img_r.shape[:2])[::-1]):
                start_point_r = start_point_r.astype(int)
                print(start_point_r)
                cv2.circle(img_r, start_point_r, 10, color=(0, 0, 255), thickness=3, )
                cv2.imshow('hairs rotated', img_r)
                cv2.waitKey(0)

    return img_r, start_points_r, end_points_r


def angle_between_2_vector(v1, v2):
    assert len(v1) == len(v2)
    num_point = len(v1) if len(v1.shape) > 1 else 1
    rotation_matrix_90_counter_clockwise = np.array([[0, -1],
                                                     [1, 0]])
    v1, v2 = v1.astype(float), v2.astype(float)
    # v1, v2 = v1.reshape((-1, 2)).T, v2.reshape((-1, 2)).T
    v1, v2 = v1 / np.linalg.norm(v1, axis=-1), v2 / np.linalg.norm(v2, axis=-1)
    v1, v2 = v1.reshape((num_point, 2, 1)), v2.reshape((num_point, 2, 1))

    v1_normal = np.matmul(rotation_matrix_90_counter_clockwise, v1)
    V1 = np.concatenate([v1, v1_normal], axis=-1)

    v2_normal = np.matmul(rotation_matrix_90_counter_clockwise, v2)
    V2 = np.concatenate([v2, v2_normal], axis=-1)

    # V1, V2 = V1.reshape((-1, 2, 2)), V2.reshape((-1, 2, 2))
    rot_from_v1_2_v2 = np.matmul(V2, np.linalg.inv(V1))
    rot = np.asarray([np.eye(3) for _ in range(num_point)])
    rot[:, :2, :2] = rot_from_v1_2_v2
    angles = [t3d.euler.mat2euler(rot[i])[-1] for i in range(num_point)] if num_point > 1 else \
    t3d.euler.mat2euler(rot[0])[-1]
    # angles = rot[:, 0, 0] if num_point > 1 else rot[0, 0, 0]
    return angles


def xyxy_2_corners_coord(x_min, y_min, x_max, y_max):
    # top_left, top_right, low_right, low_left,
    img_box_coord_tgt = np.array([
        [x_min, y_min, 1],
        [x_max, y_min, 1],
        [x_max, y_max, 1],
        [x_min, y_max, 1],
    ])
    return img_box_coord_tgt


def corners_2_bounding_box_xyxy(img_box_coord_tgt):
    # assert img_box_coord_tgt[0, 0] == img_box_coord_tgt[-1, 0], 'x_min ' + str(img_box_coord_tgt[0, 0]) + ' not equals to ' + str(img_box_coord_tgt[-1, 0])
    # assert img_box_coord_tgt[0, 1] == img_box_coord_tgt[1, 1], 'y_min ' + str(img_box_coord_tgt[0, 1]) + ' not equals to ' + str(img_box_coord_tgt[1, 1])
    # assert img_box_coord_tgt[1, 0] == img_box_coord_tgt[2, 0], 'x_max ' + str(img_box_coord_tgt[1, 0]) + ' not equals to ' + str(img_box_coord_tgt[2, 0])
    # assert img_box_coord_tgt[2, 1] == img_box_coord_tgt[3, 1], 'y_max ' +  str(img_box_coord_tgt[2, 1]) + ' not equals to ' + str(img_box_coord_tgt[3, 1])

    x_min, x_max = np.min(img_box_coord_tgt[:, 0]), np.max(img_box_coord_tgt[:, 0])
    y_min, y_max = np.min(img_box_coord_tgt[:, 1]), np.max(img_box_coord_tgt[:, 1])
    return int(x_min), int(y_min), int(x_max), int(y_max)


def merge_box(boxes):
    box_biggest = boxes[0]
    for box in boxes:
        box_biggest = (min(box_biggest[0], box[0]), min(box_biggest[1], box[1]), max(box_biggest[2], box[2]), max(box_biggest[3], box[3]))
    return box_biggest


def nearest_points_2_lines(lines, pts, normal_distance_min=3.0):
    """
    find line in
    :param pts:
    :param lines:
    :return:
    """
    normal_distance_min_square = normal_distance_min ** 2
    lines = lines / np.linalg.norm(lines, axis=-1)[:, None]  # normalize line
    pts_square = np.sum(np.power(pts, 2), axis=-1)
    index_lines_2_pts = np.asarray([-1] * len(lines))
    mask = []
    for i_line, line in enumerate(lines):
        '''compute pts projection normal to the line'''
        pts_project_on_line_square = np.power(np.matmul(pts, line), 2)
        pts_project_on_line_normal_square = pts_square - pts_project_on_line_square
        i_pt_min = np.argmin(pts_project_on_line_normal_square)
        index_lines_2_pts[i_line] = int(i_pt_min)
        mask.append(pts_project_on_line_normal_square[i_pt_min] < normal_distance_min_square)
    return index_lines_2_pts[mask].tolist(), lines[mask]


def closet_vector_2_point(lines, pts):
    """
    compute each pt projection on each line, assume line is normalized to length 1
    :param pts:
    :param lines:
    :return:
    """
    assert len(pts) == len(lines)
    lengths = np.sum(pts * lines, axis=-1)[:, None]
    return lengths * lines


def nearest_neighbor_points_2_points(pts0, pts1, distance_min=3):
    """

    :param pts0:
    :param pts1:
    :return:
    """
    query = pts0
    tree = scipy.spatial.kdtree.KDTree(pts1)
    id_picked_tree_2_dd = dict()

    id_pts1_2_pts2 = []

    for i_query, pt in enumerate(query):
        dd, id_tree_node = tree.query(pt)

        if dd > distance_min:               # max distance check
            continue

        if id_picked_tree_2_dd.get(id_tree_node) is not None:    # pick single match
            if dd > id_picked_tree_2_dd.get(id_tree_node)['dis']:
                continue
            else:   # get rid of old one
                id_pts1_2_pts2[id_picked_tree_2_dd.get(id_tree_node)['id']] = [-1, -1]

        id_picked_tree_2_dd[id_tree_node] = {'id': len(id_pts1_2_pts2), 'dis': dd}
        id_pts1_2_pts2.append([i_query, id_tree_node])
    if len(id_pts1_2_pts2) == 0:
        return []
    else:
        id_pts1_2_pts2 = np.asarray(id_pts1_2_pts2).astype(int)
        return id_pts1_2_pts2[id_pts1_2_pts2[:, 0] > 0]


# def match_filter_pts_pair(kp1, des1, kp2, des2):
def epipolar_geometry_filter_matched_pts_pair(pts1, pts2, img1=None, img2=None, flag_output=False):
    """

    :param pts1:
    :param des1:
    :param pts2:
    :param des2:
    :param img1:
    :param img2:
    :param flag_output:
    :return:
    """
    if pts1 is None or len(pts1):
        return None, None, None
    if pts2 is None or len(pts2):
        return None, None, None
    assert len(pts1) == len(pts2)
    len_input_pts = len(pts1)

    '''filter with epi-polar geometry ransac'''
    fundamental_matrix, mask = cv2.findFundamentalMat(pts1, pts2, cv2.cv2.FM_RANSAC, ransacReprojThreshold=8.0, confidence=0.9999,
                                         maxIters=10000)

    # TODO: fail case fundamental found nothing

    mask_ = mask.squeeze().astype(bool).tolist()
    pts1 = pts1[mask_]
    pts2 = pts2[mask_]

    if flag_output:
        print('epi-polar geometry ransac filter', len(pts1), '/', len_input_pts, 'points left.\n'
                                                                                 'epi-polar rms',
              np.mean(np.sum(
                  np.matmul(np.hstack([pts1, np.ones((len(pts1), 1))]), fundamental_matrix) * np.hstack(
                      [pts2, np.ones((len(pts1), 1))]),
                  axis=1)))
    if img1 is not None and img2 is not None:
        img3 = vis.draw_matches(img1, pts1, img2, pts2)
        cv2.namedWindow('hair_close_range_match', cv2.WINDOW_NORMAL)
        cv2.imshow('hair_close_range_match', img3)
        cv2.waitKey(0)
    return pts1, pts2, mask_


def main():
    lines = np.array([[1, 0, 0],
                      [0, 1, 0],
                      [0, 0, 1],
                      [1, 1, 0]])
    pts = np.copy(lines) * 9
    index_lines_2_pts, lines = nearest_points_2_lines(lines, pts)

    print(lines)
    print(pts)
    print(index_lines_2_pts)

    for i, index_line_2_pt in enumerate(index_lines_2_pts):
        print(lines[i], pts[index_line_2_pt])

    pts = pts[index_lines_2_pts]
    lines_proj = closet_vector_2_point(lines, pts)

    print(lines)
    print(pts)
    print(lines_proj)


# def main():
#     xyxy = (0, 0, 100, 100)
#     corners = xyxy_2_corners_coord(*xyxy)
#     print(corners)
#     xyxy = corners_2_bounding_box_xyxy(corners)
#     print(xyxy)


if __name__ == '__main__':
    main()
