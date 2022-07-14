# -*- coding: utf-8 -*-

"""
@author: Cheng Chen
@email: chengc0611@gmail.com
@time: 9/2/21 4:39 PM
"""
import warnings

import cv2
import numpy as np

import slam_lib.format


def draw_pts(img, pts):
    if len(img.shape) < 3:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    img_pts = np.copy(img).astype(np.uint8)

    if len(pts) == 0:
        return img_pts

    '''format pts'''
    pts = np.asarray(pts)
    if pts.shape[1] != 2:
        pts = pts.reshape(-1, 2)
    pts = pts.astype(int)

    '''draw'''
    w, h = img.shape[0], img.shape[1]
    radius = max(int(max(w, h) / 500), 5)
    thickness = max(int(radius / 2), 2)
    for i in range(len(pts)):
        pt1 = pts[i]
        color = np.random.random(3) * 255
        img_pts = cv2.circle(img_pts, pt1, radius=radius, color=color, thickness=thickness)
    return img_pts


def draw_lines(img, info, name):
    # TODO:
    for hair_info in info:
        start, end = hair_info['follicle'], hair_info['hair_end']
        cv2.line(img, start, np.asarray(start)+1, color=(255, 0, 0), thickness=3)   # root
        cv2.line(img, end, np.asarray(start), color=(0, 255, 0), thickness=1)       # hair
        cv2.line(img, end, np.asarray(end)+1, color=(0, 0, 255), thickness=3)       # head
    cv2.imshow(name+'hairs', img)
    cv2.waitKey(0)


def draw_matches_(img0, pts0, img1, pts1, show_num_match=-1):
    """"""
    '''preprocess pts'''
    pts0 = np.asarray(pts0)
    pts1 = np.asarray(pts1)

    '''preprocess images'''
    if img0 is None or img1 is None:
        warnings.warn('empty img, get ' + str(img0) + str(img1))
    if len(img0.shape) < 3:
        img0 = cv2.cvtColor(img0, cv2.COLOR_GRAY2BGR)
    if len(img1.shape) < 3:
        img1 = cv2.cvtColor(img1, cv2.COLOR_GRAY2BGR)

    width_synthesis = img0.shape[1] + img1.shape[1]
    height_synthesis = max(img0.shape[0], img1.shape[0])
    img_synthesis = np.zeros((height_synthesis, width_synthesis, 3)).astype(np.uint8)
    img_synthesis[0:img0.shape[0], 0:img0.shape[1], :] = img0
    img_synthesis[0:img1.shape[0], img0.shape[1]:, :] = img1

    if pts0 is None or len(pts1) == 0 or pts1 is None or len(pts1) == 0:
        return img_synthesis

    if not len(pts0) == len(pts1):
        warnings.warn('expect same number of pts, get' + str(len(pts0)) + str(len(pts0)))
        return img_synthesis

    pts0, pts1 = slam_lib.format.pts_2d_format(pts0), slam_lib.format.pts_2d_format(pts1)
    pts0, pts1 = pts0.astype(int), pts1.astype(int)
    pts1[:, 0] = pts1[:, 0] + img0.shape[1]

    img_synthesis_draw_match = np.copy(img_synthesis)
    w, h = img_synthesis_draw_match.shape[0], img_synthesis_draw_match.shape[1]
    radius = max(int(max(w, h) / 1000), 5)
    thickness = max(int(radius / 2), 2)
    for i_start in range(len(pts0)):
        pt1, pt2 = pts0[i_start], pts1[i_start]
        color = np.random.random(3) * 255
        img_synthesis_draw_match = cv2.circle(img_synthesis_draw_match, pt1, radius=radius, color=color,
                                              thickness=thickness)
        img_synthesis_draw_match = cv2.circle(img_synthesis_draw_match, pt2, radius=radius, color=color,
                                              thickness=thickness)
        img_synthesis_draw_match = cv2.line(img_synthesis_draw_match, pt1, pt2, color=color, thickness=int(thickness/2))

        # img_synthesis_draw_match_list = []
        # for i_start in range(0, len(pts0), show_num_match):
        #     img_synthesis_draw_match = np.copy(img_synthesis)
        #     for i_ in range(i_start, i_start+show_num_match):
        #         pt1, pt2 = pts0[i_], pts1[i_]
        #         color = np.random.random(3) * 255
        #         img_synthesis_draw_match = cv2.circle(img_synthesis_draw_match, pt1, radius=15, color=color, thickness=5)
        #         img_synthesis_draw_match = cv2.circle(img_synthesis_draw_match, pt2, radius=15, color=color, thickness=5)
        #         img_synthesis_draw_match = cv2.line(img_synthesis_draw_match, pt1, pt2, color=color, thickness=4)
        #     img_synthesis_draw_match_list.append(img_synthesis_draw_match)
        # return img_synthesis_draw_match_list
    return img_synthesis_draw_match


def draw_matches(img1, pts1, img2, pts2, horizontal=True, flag_count_match=False, show_num_match=-1):
    if pts1 is None or pts2 is None:
        pts1, pts2 = [[0, 0]], [[0, 0]]
    assert len(pts1) == len(pts2)

    if isinstance(pts1, list):
        pts1 = np.asarray(pts1)
    if isinstance(pts2, list):
        pts2 = np.asarray(pts2)

    if len(img1.shape) < 3:
        img1 = cv2.cvtColor(img1, cv2.COLOR_GRAY2BGR)
    if len(img2.shape) < 3:
        img2 = cv2.cvtColor(img2, cv2.COLOR_GRAY2BGR)
    if pts1.shape[1] != 2:
        pts1 = pts1.reshape(-1, 2)
    if pts2.shape[1] != 2:
        pts2 = pts2.reshape(-1, 2)

    # compute mapp from two image to new synthesis img
    img_synthesis = None
    if horizontal:
        width_synthesis = img1.shape[1] + img2.shape[1]
        height_synthesis = max(img1.shape[0], img2.shape[0])
        img_synthesis = np.zeros((height_synthesis, width_synthesis, 3)).astype(np.uint8)
        img_synthesis[0:img1.shape[0], 0:img1.shape[1], :] = img1
        img_synthesis[0:img2.shape[0], img1.shape[1]:, :] = img2
        pts1, pts2 = pts1.astype(int), pts2.astype(int)
        pts2[:, 0] = pts2[:, 0] + img1.shape[1]

    else:
        width_synthesis = max(img1.shape[1], img2.shape[1])
        height_synthesis = img1.shape[0] + img2.shape[0]
        img_synthesis = np.zeros((height_synthesis, width_synthesis, 3)).astype(np.uint8)
        img_synthesis[0:img1.shape[0], 0:img1.shape[1], :] = img1
        img_synthesis[img1.shape[0]:, 0:img2.shape[1], :] = img2
        pts1, pts2 = pts1.astype(int), pts2.astype(int)
        pts2[:, 1] = pts2[:, 1] + img1.shape[0]

    # img_synthesis = img_synthesis / 255
    if show_num_match == -1 or min(len(pts1), len(pts2)) < show_num_match:
        img_synthesis_draw_match = np.copy(img_synthesis)
        for i_start in range(len(pts1)):
            pt1, pt2 = pts1[i_start], pts2[i_start]
            color = np.random.random(3) * 255
            # color = color.astype(int)
            img_synthesis_draw_match = cv2.circle(img_synthesis_draw_match, pt1, radius=15, color=color,
                                                  thickness=5)
            img_synthesis_draw_match = cv2.circle(img_synthesis_draw_match, pt2, radius=15, color=color,
                                                  thickness=5)
            img_synthesis_draw_match = cv2.line(img_synthesis_draw_match, pt1, pt2, color=color, thickness=4)

    else:
        img_synthesis_draw_match_list = []
        for i_start in range(0, len(pts1), show_num_match):
            img_synthesis_draw_match = np.copy(img_synthesis)
            for i_ in range(i_start, i_start+show_num_match):
                pt1, pt2 = pts1[i_], pts2[i_]
                color = np.random.random(3) * 255
                img_synthesis_draw_match = cv2.circle(img_synthesis_draw_match, pt1, radius=15, color=color, thickness=5)
                img_synthesis_draw_match = cv2.circle(img_synthesis_draw_match, pt2, radius=15, color=color, thickness=5)
                img_synthesis_draw_match = cv2.line(img_synthesis_draw_match, pt1, pt2, color=color, thickness=4)
            img_synthesis_draw_match_list.append(img_synthesis_draw_match)
        return img_synthesis_draw_match_list

    # if flag_count_match:
    #     right_count = 0
    #     wrong_count = 0
    #     for i_start in range(len(pts1)):
    #         img_synthesis_draw_one_match = np.copy(img_synthesis)
    #         pt1, pt2 = pts1[i_start], pts2[i_start]
    #         color = np.asarray((0.0, 0.0, 1.0)) * 255.0
    #         img_synthesis_draw_one_match = cv2.circle(img_synthesis_draw_one_match, pt1, radius=15, color=color, thickness=5)
    #         img_synthesis_draw_one_match = cv2.circle(img_synthesis_draw_one_match, pt2, radius=15, color=color, thickness=5)
    #         img_synthesis_draw_one_match = cv2.line(img_synthesis_draw_one_match, pt1, pt2, color=color, thickness=2)
    #         cv2.namedWindow('hair_close_range_match', cv2.WINDOW_NORMAL)
    #         cv2.imshow('hair_close_range_match', img_synthesis_draw_one_match)
    #         waitkey = cv2.waitKey(0)
    #         if waitkey == 49:
    #             right_count += 1
    #         elif waitkey == 48:
    #             wrong_count += 1
    #         else:
    #             raise IOError('wrong input. expect 0 or 1, got ' + str(waitkey))
    #         print(right_count, wrong_count)
    #     return img_synthesis_draw_match, right_count, wrong_count
    return img_synthesis_draw_match


def main():
    return


if __name__ == '__main__':
    main()
