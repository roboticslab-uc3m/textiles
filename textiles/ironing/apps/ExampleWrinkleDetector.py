# -*- coding: utf-8 -*-
"""
ExampleWrinkleDetector computes ironing paths from wrinkle data
"""

import os

import begin

from textiles.ironing.perception.WrinkleDetection import detect_wrinkles_from_file


data_folder_pattern = "~/Research/datasets/2017-02-27-ironing-kinfu/{}"
data_file_name = "textured_mesh.ply-unsegmented.pcd-cluster1.pcd-output.pcd"
garment = 'standing-three'

data_folder = data_folder_pattern.format(garment)
data_file_path = os.path.join(os.path.abspath(os.path.expanduser(data_folder)), data_file_name)


__author__ = 'def'


@begin.start(auto_convert=True)
@begin.logging
def main(input_file, debug=False):
    input_file_absolute = os.path.abspath(os.path.expanduser(input_file))
    trajectory, descriptor = detect_wrinkles_from_file(input_file_absolute, debug=True)
    print(trajectory)
    print(descriptor)


