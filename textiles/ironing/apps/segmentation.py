import os
import logging

import begin
import numpy as np
import pypcd
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans

from textiles.common.math import normalize

def decode_rgb_from_pcl(rgb):
    """
    Convert the rgb float32 field to R, G, B uint8 values

    This function has been copied (forked) from pypcd
    :param rgb: Numpy array with RGB float field
    :return: List of RGB lists
    """
    rgb = rgb.copy()
    rgb.dtype = np.uint32
    r = np.asarray((rgb >> 16) & 255, dtype=np.uint8)
    g = np.asarray((rgb >> 8) & 255, dtype=np.uint8)
    b = np.asarray(rgb & 255, dtype=np.uint8)
    rgb_arr = np.zeros((len(rgb), 3), dtype=np.uint8)
    rgb_arr[:, 0] = r
    rgb_arr[:, 1] = g
    rgb_arr[:, 2] = b
    return rgb_arr

def encode_rgb_for_pcl(rgb):
    """
    Convert an array with R, G, B uint8 values to a rgb float32 field

    This function has been copied (forked) from pypcd
    :param rgb: Nx3 uint8 array with R, G, B values
    :return: Nx1 float32 array with bit-packed RGB, for PCL.
    """
    assert(rgb.dtype == np.uint8)
    assert(rgb.ndim == 2)
    assert(rgb.shape[1] == 3)
    rgb = rgb.astype(np.uint32)
    rgb = np.array((rgb[:, 0] << 16) | (rgb[:, 1] << 8) | (rgb[:, 2] << 0),
                   dtype=np.uint32)
    rgb.dtype = np.float32
    return rgb


@begin.start(auto_convert=True)
@begin.logging
def main(input):
    src_file = os.path.abspath(os.path.expanduser(input))

    # Load point cloud
    point_cloud = pypcd.PointCloud.from_path(src_file)

    # Unpack data
    data = point_cloud.pc_data
    X = np.array([[x, y, z, r, g, b] for (x, y, z, (r, g, b)) in zip(data['x'], data['y'], data['z'],
                                                                     decode_rgb_from_pcl(data['rgb']))])
    X_color = np.array([[r, g, b] for (r, g, b) in decode_rgb_from_pcl(data['rgb'])])

    # Color to HSV
    rgb_image = np.reshape(X[:, 3:].astype(np.uint8), (X.shape[0], 1, 3))
    from skimage.color import rgb2hsv
    hsv_image = rgb2hsv(rgb_image)
    hsv = np.reshape(hsv_image.astype(np.float32), (X.shape[0], 3))

    # Normalization
    X_norm = X.copy()
    X_norm[:, 0] = normalize(X[:, 0])
    X_norm[:, 1] = normalize(X[:, 1])
    X_norm[:, 2] = normalize(X[:, 2])
    # X_norm[:, 3:] = X[:, 3:] / 255.0

    #X_norm[:, :3] *= 1000

    X_norm[:, 3:] = hsv

    # K-means clustering
    km = KMeans(n_clusters=2)  # we know beforehand that only a garment and the ironing table exist
    km.fit(X_norm)
    labels = km.labels_

    cluster_1 = X[labels == 0]
    cluster_2 = X[labels == 1]

    # Pack data
    for i, cluster in enumerate([cluster_1, cluster_2]):
        rgb_unencoded = cluster[:, 3:]
        rgb = encode_rgb_for_pcl(rgb_unencoded.astype(np.uint8))
        packed_array = np.zeros((cluster.shape[0], 4), dtype=np.float32)
        packed_array[:, :3] = cluster[:, :3]
        packed_array[:, 3] = rgb
        point_cloud_output = pypcd.make_xyz_rgb_point_cloud(packed_array)
        point_cloud_output.save_pcd('out{}.pcd'.format(i), compression='binary_compressed')
