# -*- coding: utf-8 -*-

"""
@author: Cheng Chen
@email: chengc0611@gmail.com
@time: 12/9/21 1:40 PM
"""
import os
# import pyheif
from PIL import Image

img_file_format = {'bmp', 'BMP', 'ppm', 'jpg', 'pgm', 'JPG'}


def get_robots_ox_ac_uk_vgg_data(dataset_dir_path):
    """Affine Covariant Features Datasets"""
    img_paths = []
    homo_paths = []
    for file_name in os.listdir(dataset_dir_path):
        if '.bmp' in file_name or 'ppm' in file_name or 'jpg' in file_name or 'pgm' in file_name:
            img_paths.append(dataset_dir_path + '/' + file_name)
        else:
            homo_paths.append(dataset_dir_path + '/' + file_name)
    img_paths.sort(), homo_paths.sort()
    return img_paths, homo_paths


def get_iphone_data(dataset_dir_path):
    img_paths = []
    for file_name in os.listdir(dataset_dir_path):
        if '.bmp' in file_name or \
                'ppm' in file_name or \
                'JPG' in file_name or \
                'jpg' in file_name or \
                'pgm' in file_name:
            img_paths.append(dataset_dir_path + '/' + file_name)
    img_paths.sort()
    return img_paths


def get_iphone_data_heic(dataset_dir_path):
    img_paths = []
    img_names = os.listdir(dataset_dir_path)
    img_names_set = set(img_names)
    for file_name in os.listdir(dataset_dir_path):
        if 'heic' in file_name or \
                'HEIC' in file_name:

            file_name_jpg = file_name[:-4] + 'jpg'
            if file_name_jpg not in img_names_set:
                # reading
                heif_file = pyheif.read(dataset_dir_path + '/' + file_name)

                # convert
                img = Image.frombytes(heif_file.mode, heif_file.size, heif_file.data, 'raw', heif_file.mode,
                                      heif_file.stride)

                # saving
                img.save(dataset_dir_path + '/' + file_name_jpg, 'JPEG')

            # record
            img_paths.append(dataset_dir_path + '/' + file_name_jpg)
    img_paths.sort()
    return img_paths


def get_triple_camera_data(dataset_dir_path):
    mono_dir_name = '/mono/'
    stereo_dir_name = '/stereo/'
    stereo_left_dir_name = '/left/'
    stereo_right_dir_name = '/right/'

    mono_img_path = []
    stereo_img_path = {'left': [], 'right': []}

    for file_name in os.listdir(dataset_dir_path + mono_dir_name):
        if file_name[-3:] in img_file_format:
            mono_img_path.append(dataset_dir_path + mono_dir_name + '/' + file_name)

    for file_name in os.listdir(dataset_dir_path + stereo_dir_name + stereo_left_dir_name):
        if file_name[-3:] in img_file_format:
            stereo_img_path['left'].append(dataset_dir_path + stereo_dir_name + stereo_left_dir_name + '/' + file_name)

    for file_name in os.listdir(dataset_dir_path + stereo_dir_name + stereo_right_dir_name):
        if file_name[-3:] in img_file_format:
            stereo_img_path['right'].append(
                dataset_dir_path + stereo_dir_name + stereo_right_dir_name + '/' + file_name)

    mono_img_path.sort()
    stereo_img_path['left'].sort()
    stereo_img_path['right'].sort()
    return mono_img_path, stereo_img_path


class Dataset:
    def __init__(self):
        # some data path naming convensions
        self.calibration_img_dir_name = '/calibration/'
        self.general_img_dir_name = '/img/'
        self.left_img_dir_name = '/left/'
        self.right_img_dir_name = '/right/'

        self.left_calibration_img = []
        self.right_calibration_img = []
        self.left_general_img = []
        self.right_general_img = []

    def load_from_dir(self, data_dir_path):
        if not os.path.isdir(data_dir_path):
            print('data dir not available')
            return

        if os.path.isdir(data_dir_path + self.calibration_img_dir_name):
            self.left_calibration_img, self.right_calibration_img = get_left_right_img_path_in_two_folder(
                data_dir_path + self.calibration_img_dir_name + self.left_img_dir_name,
                data_dir_path + self.calibration_img_dir_name + self.right_img_dir_name)
        else:
            print('no calibration data found')

        if os.path.isdir(data_dir_path + self.general_img_dir_name):
            self.left_general_img, self.right_general_img = get_left_right_img_path_in_two_folder(
                data_dir_path + self.general_img_dir_name + self.left_img_dir_name,
                data_dir_path + self.general_img_dir_name + self.right_img_dir_name)
        else:
            print('no general data found')


def get_left_right_img_path_in_two_folder(left_img_dir, right_img_dir):
    left_img_paths, right_img_paths = [], []
    left_img_names = sorted(os.listdir(left_img_dir))
    right_img_names = sorted(os.listdir(right_img_dir))
    for left_img_name, right_img_name in zip(left_img_names, right_img_names):
        # check left img time stamp earlier than right img
        assert left_img_names < right_img_names, 'Timestamp error: expect left img taken earlier than right img, ' \
                                                 'but get left img as < ' + str(left_img_names) + '> right img as < ' \
                                                 + str(right_img_names) + '>'

        left_img_paths.append(left_img_dir + '/' + left_img_name)
        right_img_paths.append(right_img_dir + '/' + right_img_name)
    return left_img_paths, right_img_paths


def get_left_right_img_path_in_one_folder(img_dir):
    img_paths_left, img_paths_right = [], []
    if not os.path.isdir(img_dir):
        return [], []
    img_names = sorted(os.listdir(img_dir))
    for img_name in img_names:
        if img_name[:4] == 'left':
            img_paths_left.append(img_dir + '/' + img_name)
            img_paths_right.append(img_dir + '/' + 'righ' + img_name[len('lef'):])
    return img_paths_left, img_paths_right


def get_calibration_and_img(data_dir,
                            calibration_img_dir_name='/calibration/',
                            general_img_dir_name='/img/',
                            left_img_dir_name='/left/',
                            right_img_dir_name='/right/'):

    data = {'left_calibration_img': [],
            'right_calibration_img': [],
            'left_general_img': [],
            'right_general_img': []}

    if not os.path.isdir(data_dir):
        print('data dir not available')
        return data

    if os.path.isdir(data_dir + calibration_img_dir_name):
        data['left_calibration_img'], data['right_calibration_img'] = get_left_right_img_path_in_two_folder(
            data_dir + calibration_img_dir_name + left_img_dir_name,
            data_dir + calibration_img_dir_name + right_img_dir_name)

    else:
        print('no calibration data found')

    if os.path.isdir(data_dir + general_img_dir_name):
        data['left_general_img'], data['right_general_img'] = get_left_right_img_path_in_two_folder(
            data_dir + general_img_dir_name + left_img_dir_name,
            data_dir + general_img_dir_name + right_img_dir_name)
    else:
        print('no general data found')

    return data


def main():
    # ############################## robots_ox_ac_uk_vgg_data ##############################
    # img_paths, homo_paths = get_robots_ox_ac_uk_vgg_data('../data/graf')
    # print('############################## robots_ox_ac_uk_vgg_data ##############################')
    # print(img_paths)
    # print(homo_paths)
    # print()
    #
    # ############################## iphone ##############################
    # img_paths = get_iphone_data('../data/head')
    # print('############################## iphone ##############################')
    # print(img_paths)
    # print()

    ############################## iphone raw ##############################
    img_paths = get_iphone_data_heic('../data/tank')
    print('############################## iphone ##############################')
    print(img_paths)
    print()


if __name__ == '__main__':
    main()
