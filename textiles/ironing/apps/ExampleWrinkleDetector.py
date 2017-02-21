# -*- coding: utf-8 -*-
"""
ExampleWrinkleDetector computes ironing paths from wrinkle data
"""

import os

from textiles.ironing.perception.WrinkleDetection import detect_wrinkles_from_file


data_folder_pattern = "~/Research/datasets/2017-02-13-ironing-experiments/{}"
data_file_name = "textured_mesh.ply-output.pcd"
garment = 'hoodie-01'

data_folder = data_folder_pattern.format(garment)
data_file_path = os.path.join(os.path.abspath(os.path.expanduser(data_folder)), data_file_name)


__author__ = 'def'

if __name__ == '__main__':
    trajectory = detect_wrinkles_from_file(data_file_path, debug=True)
    print(trajectory)


