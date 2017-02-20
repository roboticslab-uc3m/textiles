import os
import logging

import begin
import numpy as np
import pypcd
from sklearn.cluster import KMeans
from skimage.color import rgb2hsv

from textiles.common.math import normalize


def segmentation_on_point_cloud_file(input_cloud_file):
    src_file = os.path.abspath(os.path.expanduser(input))

    # Load point cloud
    point_cloud = pypcd.PointCloud.from_path(src_file)

    # Perform segmentation
    output_cloud1, output_cloud2 = segmentation_on_point_cloud(point_cloud)

    # Save point cloud to file
    for i, cluster in enumerate([output_cloud1, output_cloud2]):
        rgb_unencoded = cluster[:, 3:]
        rgb = pypcd.encode_rgb_for_pcl(rgb_unencoded.astype(np.uint8))
        packed_array = np.zeros((cluster.shape[0], 4), dtype=np.float32)
        packed_array[:, :3] = cluster[:, :3]
        packed_array[:, 3] = rgb
        point_cloud_output = pypcd.make_xyz_rgb_point_cloud(packed_array)
        point_cloud_output.save_pcd('out{}.pcd'.format(i), compression='binary_compressed')


def segmentation_on_point_cloud(input_point_cloud):
    # Unpack data
    data = input_point_cloud.pc_data
    X = np.array([[x, y, z, r, g, b] for (x, y, z, (r, g, b)) in zip(data['x'], data['y'], data['z'],
                                                                     pypcd.decode_rgb_from_pcl(data['rgb']))])

    # Color to HSV
    rgb_image = np.reshape(X[:, 3:].astype(np.uint8), (X.shape[0], 1, 3))
    hsv_image = rgb2hsv(rgb_image)
    hsv = np.reshape(hsv_image.astype(np.float32), (X.shape[0], 3))

    # Normalization
    X_norm = X.copy()
    X_norm[:, 0] = normalize(X[:, 0])
    X_norm[:, 1] = normalize(X[:, 1])
    X_norm[:, 2] = normalize(X[:, 2])
    # X_norm[:, 3:] = X[:, 3:] / 255.0

    # X_norm[:, :3] *= 1000

    X_norm[:, 3:] = hsv

    # K-means clustering
    km = KMeans(n_clusters=2)  # we know beforehand that only a garment and the ironing table exist
    km.fit(X_norm)
    labels = km.labels_

    cluster_1 = X[labels == 0]
    cluster_2 = X[labels == 1]

    return cluster_1, cluster_2


@begin.start(auto_convert=True)
@begin.logging
def main(input: 'Input point cloud'):
    segmentation_on_point_cloud_file(input)



