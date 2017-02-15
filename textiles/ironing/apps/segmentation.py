import os
import logging

import begin
import numpy as np
import pypcd
import matplotlib.pyplot as plt


@begin.start(auto_convert=True)
@begin.logging
def main(input):
    src_file = os.path.abspath(os.path.expanduser(input))

    # Load point cloud
    point_cloud = pypcd.PointCloud.from_path(src_file)

    data = point_cloud.pc_data
    logging.info([pypcd.decode_rgb_from_pcl(i) for i in data['rgb']])
    X = np.array([[i, j, k] for (i, j, k) in zip(data['x'], data['y'], data['z'])])

    fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # ax.scatter(data[:,0], data[:,1], data[:,2], c='r', marker='o')
    plt.scatter(X[:, 0], X[:, 1], c='r', marker='.')
    plt.show()
