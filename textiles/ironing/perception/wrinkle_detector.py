import os
from itertools import cycle

import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import pypcd
from sklearn import mixture

# Input file(s)
src_file = "~/Research/jresearch/2016-06-23-textiles-ironing/hoodie2/colored_mesh_1.ply-output.pcd"
src_file = os.path.abspath(os.path.expanduser(src_file))

if __name__ == "__main__":
    # Load point cloud
    point_cloud = pypcd.PointCloud.from_path(src_file)

    # center the point cloud (xy)
    point_cloud.pc_data['x'] -= point_cloud.pc_data['x'].mean()
    point_cloud.pc_data['y'] -= point_cloud.pc_data['y'].mean()

    data = point_cloud.pc_data
    X = np.array([ [i, j, k] for (i, j, k) in zip(data['x'], data['y'], data['z'])])

    fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # ax.scatter(data[:,0], data[:,1], data[:,2], c='r', marker='o')
    plt.scatter(X[:, 0], X[:, 1], c='r', marker='.')
    plt.show()

    lowest_bic = np.infty
    bic = []
    n_components_range = range(1, 27)
    cv_types = ['spherical', 'tied', 'diag', 'full']
    for cv_type in cv_types:
        for n_components in n_components_range:
            # Fit a mixture of Gaussians with EM
            gmm = mixture.GMM(n_components=n_components, covariance_type=cv_type)
            gmm.fit(X)
            bic.append(gmm.bic(X))
            if bic[-1] < lowest_bic:
                lowest_bic = bic[-1]
                best_gmm = gmm

    bic = np.array(bic)
    color_iter = cycle(['k', 'r', 'g', 'b', 'c', 'm', 'y'])
    clf = best_gmm
    bars = []

    # Plot the BIC scores
    spl = plt.subplot(2, 1, 1)
    for i, (cv_type, color) in enumerate(zip(cv_types, color_iter)):
        xpos = np.array(n_components_range) + .2 * (i - 2)
        bars.append(plt.bar(xpos, bic[i * len(n_components_range):
                                      (i + 1) * len(n_components_range)],
                            width=.2, color=color))
    plt.xticks(n_components_range)
    plt.ylim([bic.min() * 1.01 - .01 * bic.max(), bic.max()])
    plt.title('BIC score per model')
    xpos = np.mod(bic.argmin(), len(n_components_range)) + .65 +\
        .2 * np.floor(bic.argmin() / len(n_components_range))
    plt.text(xpos, bic.min() * 0.97 + .03 * bic.max(), '*', fontsize=14)
    spl.set_xlabel('Number of components')
    spl.legend([b[0] for b in bars], cv_types)

    plt.show()
