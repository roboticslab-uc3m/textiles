# coding=utf-8

"""
DepthMapToPointCloud
---------------------------------------------------------------------
Functions to convert a depth image captured from an ASUS XTion Pro Live
sensor to point cloud in PCL's PCD format.
"""

import sys

import numpy as np

intrinsic_parameters_asus = np.array([[525.0,    0.0, 319.5],
                                      [  0.0,  525.0, 239.5],
                                      [  0.0,    0.0,   1.0]])

pcd_xyz_header = """# PCD v.7 - Point Cloud Data file format
VERSION .7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH {0:d}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {0:d}
DATA ascii
"""


def depth_map_to_point_cloud(input_file, output_file):
    """
    Converts a depth map saved as text file to a point cloud stored in a PCD file
    :param input_file: depth map saved as text file
    :param output_file: point cloud stored in a PCD file
    """

    src = np.loadtxt(input_file)
    indices_x, indices_y = np.indices(src.shape)
    points = [np.array([[x], [y], [z]]) for x, y, z in zip(np.ravel(indices_x), np.ravel(indices_y), np.ravel(src))]
    point_cloud = [np.dot(intrinsic_parameters_asus, p) for p in points]

    with open(output_file, 'w') as f:
        f.write(pcd_xyz_header.format(len(point_cloud)))
        for point in point_cloud:
            f.write("{} {} {}\n".format(point[0][0], point[1][0], point[2][0]))


if __name__ == '__main__':
    if len(sys.argv) != 3:
        print("Usage: DepthMapToPointCloud.py input_file.txt output_file.pcd\n")
    else:
        _, input_file, output_file = sys.argv
        depth_map_to_point_cloud(input_file, output_file)

