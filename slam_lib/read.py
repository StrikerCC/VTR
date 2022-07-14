# -*- coding: utf-8 -*-

"""
@author: Cheng Chen
@email: chengc0611@gmail.com
@time: 9/2/21 3:46 PM
"""
import copy
import json

import cv2
import numpy as np


def format_shiwei_txt():
    in_txt_path = 'data/hair_inf.txt'
    out_json_path = 'data/hair_info.json'
    hair_info_list = []

    txt_file = open(in_txt_path, 'r')
    for line in txt_file.readlines():
        hair_info = {}
        key_values = line.split()
        key_values[0] += key_values[1]
        key_values[2] += key_values[3]
        key_values = [key_values[i] for i in [0, 2, 4, 5, 6]]
        for i, key_value in enumerate(key_values):
            key, value = key_value.split(':')
            value = json.loads(value)
            hair_info[key] = value
        hair_info_list.append(hair_info)

    with open(out_json_path, 'w') as f:
        json.dump(hair_info_list, f)


def format_data(img_path, data_json_path):
    # img_path = 'data/20210902153900.png'
    # data_json_path = 'data/hair_info.json'

    with open(data_json_path) as f:
        data = json.load(f)
    img_src = cv2.imread(img_path)
    return copy.deepcopy(img_src), data


def dic_2_nparray(img, dict_list):
    start_points, end_points = [], []
    for dict in dict_list:
        start_points.append(dict['follicle'])
        end_points.append(dict['hair_end'])
        # cv2.circle(img, start_points[-1], 10, color=(0, 0, 155), thickness=3)
        # cv2.imshow('hairs', img)
        # cv2.waitKey(0)

    return np.asarray(start_points), np.asarray(end_points)


def readtxt():
    file_path = './data/yanlin.txt'
    f = open(file_path)
    cam = []
    ct = []
    reading = cam

    for line in f.readlines():
        if '#' in line:
            reading = ct
            continue
        if line == '\n':
            continue
        key, value = line.split(' ')
        x, y, z = value.split(',')
        x, y, z = float(x), float(y), float(z)
        # print(key, x , y ,z)
        reading.append([x, y, z])
    print(ct)
    print(cam)
    return cam, ct


if __name__ == '__main__':
    # format_shiwei_txt()
    readtxt()

