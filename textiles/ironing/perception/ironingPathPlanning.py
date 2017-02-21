# -*- coding: utf-8 -*-
"""
ironingPathPlanning computes ironing paths from wrinkle data
"""

import os

import numpy as np
from skimage import img_as_ubyte
from skimage.morphology import binary_erosion, disk

from textiles.ironing.perception.WrinkleDetector import detect_wrinkles


data_folder_pattern = "~/Research/datasets/2017-02-13-ironing-experiments/{}"
data_file_name = "textured_mesh.ply-output.pcd"
depth_file_suffix = "-depth_image.m"
image_file_suffix = "-wild_image.m"
mask_file_suffix = "-image_mask.m"
garment = 'hoodie-01'

data_folder = data_folder_pattern.format(garment)
data_file_path = os.path.join(os.path.abspath(os.path.expanduser(data_folder)), data_file_name)
# image_filename = data_file_path + depth_file_suffix
image_filename = data_file_path + image_file_suffix
mask_filename = data_file_path + mask_file_suffix

__author__ = 'def'

if __name__ == '__main__':

    # Load data from files
    image = np.loadtxt(image_filename)
    mask = np.loadtxt(mask_filename)
    mask = img_as_ubyte(binary_erosion(mask), disk(7))

    trajectory = detect_wrinkles(image, mask=mask)
    print(trajectory)


