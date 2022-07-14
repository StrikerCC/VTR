# -*- coding: utf-8 -*-

"""
@author: Cheng Chen
@email: chengc0611@gmail.com
@time: 12/10/21 4:07 PM
"""
import cv2
import numpy as np


def blend_by_median(img1, img2):
    """assume img1 and img2 in same view, and img1 has bigger range"""
    if img1 is None or img2 is None:
        return None
    # mask1 = np.max(img1, axis=-1) > 0 if len(img1.shape) > 2 else img1 > 0
    # mask2 = np.max(img2, axis=-1) > 0 if len(img2.shape) > 2 else img2 > 0
    mask1 = img1 != 0
    mask2 = img2 != 0
    mask_overlapping = np.logical_and(mask1, mask2)
    img_merge = img1.copy()
    img_merge[mask_overlapping] = (img1[mask_overlapping].astype(int)/3*2*0 + img2[mask_overlapping].astype(int)/1)
    return img_merge, mask_overlapping


def main():
    img1 = cv2.imread('../data/graf/img1.ppm')

    # add white noise to make img2
    mask_white_noise = np.random.random((img1.shape[:2]))
    img2 = img1.copy()
    img2[mask_white_noise > 0.95] = (255, 255, 255)

    img3 = blend_by_median(img1, img2)

    cv2.imshow('img1', img1)
    cv2.imshow('img2', img2)
    cv2.imshow('img3', img3)
    cv2.waitKey(0)


if __name__ == '__main__':
    main()
