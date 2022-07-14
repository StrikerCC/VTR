# -*- coding: utf-8 -*-

"""
@author: Cheng Chen
@email: chengc0611@gmail.com
@time: 9/2/21 5:46 PM
"""
import copy
import time
from itertools import compress

import cv2
import numpy as np
from scipy.spatial.kdtree import KDTree

import slam_lib.geometry
import slam_lib.mapping
import slam_lib.vis as vis

import torch
import slam_lib.models
from slam_lib.models.matching import Matching
from slam_lib.models.utils import AverageTimer, read_image

wait_key = 0


class MatcherSuperGlue:
    def __init__(self, cuda='1'):
        self.timer = AverageTimer(newline=True)

        # Load the SuperPoint and SuperGlue models.
        self.device = 'cuda:'+str(cuda) if torch.cuda.is_available() else 'cpu'
        # self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        torch.set_grad_enabled(False)
        config = {
            'superpoint': {
                'nms_radius': 4,
                'keypoint_threshold': 0.005,
                'max_keypoints': 1024
            },
            'superglue': {
                'weights': 'indoor',
                'sinkhorn_iterations': 20,
                'match_threshold': 0.2,
            }
        }

        self.matching = Matching(config).eval().to(self.device)
        print('SuperGlue', config)
        print('Running inference on device \"{}\"'.format(self.device))

        # load dataset
        # self.resize = [4096, 3000]
        # self.resize = [2048, 1500]
        # self.resize = [1365, 1000]
        # self.resize = [1024, 750]
        # self.resize = [512, 375]

        # self.resize = [3088, 2064]
        # self.resize = [1544, 1032]
        self.resize = [772, 516]
        # self.resize = [386, 258]

    def match(self, img_1, img_2, flag_vis=False):
        torch.set_grad_enabled(False)

        img_left, img_right = None, None
        if isinstance(img_1, str):
            img_left = cv2.imread(img_1)
        elif isinstance(img_1, np.ndarray):
            img_left = img_1

        if isinstance(img_2, str):
            img_right = cv2.imread(img_2)
        elif isinstance(img_2, np.ndarray):
            img_right = img_2

        gray_0, inp0, scales0 = read_image(img_1, self.device, self.resize, rotation=0, resize_float=True)
        gray_1, inp1, scales1 = read_image(img_2, self.device, self.resize, rotation=0, resize_float=True)

        pred = self.matching({'image0': inp0, 'image1': inp1})

        pred = {k: v[0].cpu().numpy() for k, v in pred.items()}
        # pred = {k: v[0].detach().numpy for k, v in pred.items()}


        kpts_0, kpts_1 = pred['keypoints0'], pred['keypoints1']
        feats_0, feats_1 = pred['descriptors0'].T, pred['descriptors1'].T
        matches, conf = pred['matches0'], pred['matching_scores0']

        self.timer.update('matcher')

        # Keep the matching keypoints and scale points back
        valid = matches > -1
        mkpts_0 = kpts_0[valid]
        mfeats_0 = feats_0[valid]
        mkpts_1 = kpts_1[matches[valid]]
        mfeats_1 = feats_1[matches[valid]]
        mconf = conf[valid]

        pts_2d_0, pts_2d_1 = slam_lib.mapping.scale_pts(scales0, mkpts_0), slam_lib.mapping.scale_pts(scales1, mkpts_1)

        if flag_vis:
            img3 = slam_lib.vis.draw_matches(img_left, pts_2d_0, img_right, pts_2d_1)
            cv2.namedWindow('hair_close_range_match', cv2.WINDOW_NORMAL)
            cv2.imshow('hair_close_range_match', img3)
            cv2.waitKey(0)

        return pts_2d_0, feats_0, pts_2d_1, feats_1


def make_chessbaord_corners_coord(chessboard_size, square_size):
    chessbaord_corners_coord = np.zeros((chessboard_size[0] * chessboard_size[1], 3))
    xy = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
    chessbaord_corners_coord[:, :2] = xy
    return chessbaord_corners_coord * square_size


def get_checkboard_corners(img, checkboard_size, flag_vis=False):
    assert len(checkboard_size) == 2
    flag_found, corners = cv2.findChessboardCorners(img, checkboard_size)
    if flag_found:
        cv2.cornerSubPix(img, corners, winSize=(5, 5), zeroZone=(-1, -1),
                         criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        if flag_vis:
            cv2.namedWindow('checkboard corners', cv2.WINDOW_NORMAL)
            cv2.drawChessboardCorners(img, patternSize=checkboard_size, corners=corners, patternWasFound=True)
            cv2.imshow('checkboard corners', img)
            cv2.waitKey(0)
    return corners


def sift_features(img, flag_debug=False):
    sift = cv2.SIFT_create()
    kp, des = sift.detectAndCompute(img, None)

    if flag_debug:
        img_kp = cv2.drawKeypoints(img, kp, None)
        cv2.namedWindow('kp', cv2.WINDOW_NORMAL)
        cv2.imshow('kp', img_kp)
        cv2.waitKey(wait_key)
    pts = np.float32([kp.pt for kp in kp]).reshape(-1, 2)
    return pts, des


def get_sift_pts_and_sift_feats(img1, img2, shrink=-1.0, flag_debug=False):
    """

    :param img1:
    :param img2:
    :param shrink: -1.0: shrink bigger img to hair_close_range_match smaller img size. 0: no shrink, >0, shrink both by what ratio shrink given
    :param flag_debug:
    :return:
    """
    if img1 is None or img2 is None:
        return None
    '''shrink img'''
    shrink1, shrink2 = 1, 1
    img1_sub, img2_sub = img1, img2

    if shrink == -1.0:
        img1_over_img2 = img1.shape[0] / img2.shape[0]
        if shrink > 1.0:
            shrink1 = img1_over_img2
        else:
            shrink2 = 1.0 / img1_over_img2
    elif shrink == 0.0:
        pass
    elif shrink > 0.0:
        shrink1, shrink2 = shrink, shrink
    else:
        raise NotImplementedError('shrink #' + str(shrink) + ' not implemented')

    if not shrink1 == 1:
        img1_sub = cv2.resize(img1_sub, (int(img1.shape[1] / shrink1), int(img1.shape[0] / shrink1)))
    if not shrink2 == 1:
        img2_sub = cv2.resize(img2_sub, (int(img2.shape[1] / shrink2), int(img2.shape[0] / shrink2)))

    '''compute sift feature'''
    pts_1, des1 = sift_features(img1_sub, flag_debug)
    pts_2, des2 = sift_features(img2_sub, flag_debug)

    print(len(pts_1), len(pts_1))

    pts_1 = pts_1 * shrink1
    pts_2 = pts_2 * shrink2

    return pts_1, des1, pts_2, des2


def match_sift_feats(pts1, des1, pts2, des2, img1=None, img2=None, flag_output=False):
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

    '''filter with feature value ambiguity'''
    if pts1 is None or len(pts1):
        return None, None, None, None, None
    if pts2 is None or len(pts2):
        return None, None, None, None, None

    bf = cv2.BFMatcher_create()
    matches = bf.knnMatch(des1, des2, k=2)
    good_matches = [first for first, second in matches if first.distance < 0.65 * second.distance]
    index_match = np.asarray([[m.queryIdx, m.trainIdx] for m in good_matches])

    pts1 = pts1[index_match[:, 0].tolist()]  # queryIdx
    des1 = des1[index_match[:, 0]]
    pts2 = pts2[index_match[:, 1].tolist()]  # trainIdx
    des2 = des2[index_match[:, 1]]

    # pts1 = np.float32([pts1[m.queryIdx] for m in good_matches]).reshape(-1, 1, 2)
    # pts2 = np.float32([pts2[m.trainIdx] for m in good_matches]).reshape(-1, 1, 2)

    if flag_output:
        print('feature value ambiguity', len(good_matches), '/', len(matches), 'points left')
    if img1 is not None and img2 is not None:
        print('Get ', len(pts1), ' good matches')
        img3 = vis.draw_matches(img1, pts1, img2, pts2)
        cv2.namedWindow('hair_close_range_match', cv2.WINDOW_NORMAL)
        cv2.imshow('hair_close_range_match', img3)
        cv2.waitKey(wait_key)

    return pts1, des1, pts2, des2, index_match


def get_epipolar_geometry_filtered_sift_matched_pts(img1, img2, shrink=1.0, flag_debug=False):
    if img1 is None or img2 is None:
        return None

    pts1, des1, pts2, des2 = get_sift_pts_and_sift_feats(img1, img2, shrink=shrink, flag_debug=flag_debug)
    pts1, des1, pts2, des2, _ = match_sift_feats(pts1, des1, pts2, des2)
    pts1, pts2, _ = slam_lib.geometry.epipolar_geometry_filter_matched_pts_pair(pts1, pts2)

    '''statistics'''
    if flag_debug:
        print('Get ', len(pts1), ' good matches')
        img3 = vis.draw_matches(img1, pts1, img2, pts2)
        cv2.namedWindow('hair_close_range_match', cv2.WINDOW_NORMAL)
        cv2.imshow('hair_close_range_match', img3)
        cv2.waitKey(wait_key)

    return pts1, des1, pts2, des2


# def get_pts_pair_by_sift(img1, img2, flag_debug=False):
#     pts1, des1, pts2, des2, good_matches = get_epipolar_geometry_filtered_sift_matched_pts(img1, img2, flag_debug)
#     return pts1, pts2


def get_pts(img):
    # TODO:
    pass


def compute_fpf(img, start_points, end_points, num_neighbor):
    """"""
    # num_neighbor = 5
    num_hair = len(start_points)

    assert start_points.shape[1] == 2 and end_points.shape[1] == 2
    '''setup local frame for each point'''
    # use hair line as first axis, counter-clockwise as positive rotation

    hair_line = end_points - start_points
    hair_line_unit_vector = hair_line / np.expand_dims(np.linalg.norm(hair_line, axis=1), axis=-1)
    assert 0.999 < np.linalg.norm(hair_line_unit_vector[0]) < 1.0001, np.linalg.norm(hair_line_unit_vector[0])

    '''build tree for hair data, key is hair start coordinate, value is hair orientation'''
    tree_start = KDTree(start_points)

    '''use orientation and distance of knn feature to make extra dimension data, start from neighbor close to first axis'''
    feature_dis_start = np.zeros((num_hair, num_neighbor))
    feature_dis_end = np.zeros((num_hair, num_neighbor))
    feature_orientation_neighbor_line = np.zeros((num_hair, num_neighbor))
    feature_orientation_neighbor_hair = np.zeros((num_hair, num_neighbor))

    for index_hair in range(num_hair):
        hair_point = start_points[index_hair]
        _, nn_index_list = tree_start.query(hair_point, k=num_neighbor + 1)
        nn_index_list = nn_index_list[1:]  # get rid of point itself

        for i_nn, index_nn in enumerate(nn_index_list):
            feature_dis_start[index_hair, i_nn] = np.linalg.norm(
                start_points[index_hair] - start_points[index_nn])  # distance
            feature_dis_end[index_hair, i_nn] = np.linalg.norm(
                end_points[index_hair] - end_points[index_nn])  # distance

            feature_orientation_neighbor_line[index_hair, i_nn] = slam_lib.geometry.angle_between_2_vector(
                hair_line[index_hair], hair_line[
                    index_nn])  # orientation of line segment from hair start to neighbor hair start in local frame
            # feature_orientation_neighbor_line[index_hair, i_nn] = np.random.random(1)

        # reorganize the nn list, start from point heading close to first axis
        mask_sorted = np.argsort(feature_orientation_neighbor_line[index_hair])
        feature_dis_start[index_hair] = feature_dis_start[index_hair][mask_sorted]
        feature_dis_end[index_hair] = feature_dis_end[index_hair][mask_sorted]

        feature_orientation_neighbor_line[index_hair] = feature_orientation_neighbor_line[index_hair][mask_sorted]
        feature_orientation_neighbor_hair[index_hair] = feature_orientation_neighbor_hair[index_hair][mask_sorted]

    feature_dis_start /= np.expand_dims(np.sum(feature_dis_start, axis=1), axis=-1)  # normalize distance
    feature_dis_end /= np.expand_dims(np.sum(feature_dis_end, axis=1), axis=-1)
    # feature_dis_start *= np.pi
    # feature_dis_end *= np.pi

    feature_orientation_neighbor_line = np.arctan(
        feature_orientation_neighbor_line)  # orientation of line segment from hair start to neighbor hair start in local frame
    # feature_orientation_neighbor_line /= 2 * np.pi
    # feature_orientation_neighbor_line += 0.5

    '''build new feature tree from new feature'''
    # feature = np.concatenate([feature_dis, feature_orientation_neighbor_line, feature_orientation_neighbor_hair], axis=1)
    feature = np.concatenate([feature_dis_start, feature_orientation_neighbor_line], axis=1)
    # feature = feature / np.expand_dims(np.linalg.norm(feature, axis=-1), axis=-1)
    # feature = np.concatenate([feature_dis_start, feature_dis_end], axis=1)
    return feature


def main():
    # time_0 = time.time()
    # vis_flag = True
    # num_nn = 5
    # img_l, data_l = read.format_data(img_path='/home/cheng/proj/3d/hair_host/bin/left.jpg',
    #                                  data_json_path='/home/cheng/proj/3d/hair_host/bin/left_hair_info.json')
    # img_l_copy = copy.deepcopy(img_l)
    # vis.draw_lines(img_l_copy, data_l, 'l')
    #
    # img_r, data_r = read.format_data(img_path='/home/cheng/proj/3d/hair_host/bin/right.jpg',
    #                                  data_json_path='/home/cheng/proj/3d/hair_host/bin/right_hair_info.json')
    # img_r_copy = copy.deepcopy(img_r)
    # vis.draw_lines(img_r_copy, data_r, 'r')
    #
    # start_points_l, end_points_l = read.dic_2_nparray(img_l, data_l)
    # start_points_r, end_points_r = read.dic_2_nparray(img_r, data_r)
    # # cv2.namedWindow('line', cv2.WINDOW_NORMAL)
    #
    #
    # # siftfeatures = cv2.xfeatures2d.SIFT_create()
    # # keypoints = siftfeatures.detect(img_l_copy, None)
    # # img_l_copy = cv2.drawKeypoints(img_l_copy, keypoints=keypoints, outImage=0, color=(0, 255, 0))
    #
    #
    # img_r, start_points_r, end_points_r = utils.tsfm(img_l, start_points_l, end_points_l)
    #
    # feature_1 = compute_fpf(img_l, start_points_l, end_points_l, num_neighbor=num_nn)
    # feature_2 = compute_fpf(img_r, start_points_r, end_points_r, num_neighbor=num_nn)
    #
    # '''randomize data'''
    # mask = np.arange(0, len(feature_2)).astype(int)
    # # np.random.shuffle(mask)
    # feature_2 = feature_2[mask]
    # '''for each hair, locate it nn in new feature space'''
    # num_hair = len(feature_1)
    # num_success = 0
    # tree_feature_2 = KDTree(feature_2)
    # num_neighbor_feature = 1
    #
    # img_feature = np.concatenate([img_l, img_r], axis=1)
    # for i_hair in range(num_hair):
    #     hair_feature = feature_1[i_hair]
    #     _, nn_index_list = tree_feature_2.query(hair_feature, k=num_neighbor_feature + 1)
    #     if mask[nn_index_list[0]] == i_hair:
    #         num_success += 1
    #     if vis_flag:
    #         print(i_hair)
    #         # for index_nn in nn_index_list:
    #         index_match = mask[nn_index_list[0]]
    #         print('     ', index_match)
    #         start_point_r = start_points_r[index_match].astype(int)
    #         start_point_r += np.asarray([img_l.shape[1], 0])
    #         cv2.line(img_feature, start_points_l[i_hair], start_point_r, color=(0, 0, 155), thickness=2)
    #
    # cv2.namedWindow('line', cv2.WINDOW_NORMAL)
    # cv2.imshow('line', img_feature)
    # cv2.waitKey(0)
    #     # cv2.destroyAllWindows()
    #
    # '''out'''
    # print('success rate', num_success/num_hair, 'time', time.time() - time_0)
    return


if __name__ == '__main__':
    main()
