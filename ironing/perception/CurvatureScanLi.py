# coding=utf-8

import os
from operator import itemgetter
from skimage import io
from skimage import feature
from skimage import morphology
import numpy as np
from sklearn.mixture import GaussianMixture
import matplotlib as mpl
import matplotlib.pyplot as plt

"""
Curvature Scan Li
---------------------------------------------------------------------
Curvature scan as described in Yinxiao Li's Thesis and in article:
 * Y. Li, X. Hu, D. Xu, Y. Yue, E. Grinspun, and P. Allen, “Multi-
Sensor Surface Analysis for Robotic Ironing,” in IEEE International
Conference on Robotics and Automation (ICRA), Stockholm, 2016.

Finds height bumps on garments based on depth data.
"""

image_folder = "~/Research/jresearch/2016-10-10-replicate-li/"
image_folder = os.path.abspath(os.path.expanduser(image_folder))
image_name_pattern = "garment-{}-depth.ppm"


def shape_index_filter(img, lower_limit, upper_limit, hessian_sigma=1):
    Hxx, Hxy, Hyy = feature.hessian_matrix(img, sigma=hessian_sigma)
    k1, k2 = feature.hessian_matrix_eigvals(Hxx, Hxy, Hyy)
    shape_index = 2 / np.pi * np.arctan((k1 + k2) / (k1 - k2))
    wrinkles_low = np.where(lower_limit <= shape_index, 1, 0)
    wrinkles_upper = np.where(shape_index < upper_limit, 1, 0)
    return np.bitwise_and(wrinkles_low, wrinkles_upper)


colors = ['navy', 'turquoise', 'darkorange']

def make_ellipses(gmm, ax):
    for n, color in enumerate(colors):
        if gmm.covariance_type == 'full':
            covariances = gmm.covariances_[n][:2, :2]
        elif gmm.covariance_type == 'tied':
            covariances = gmm.covariances_[:2, :2]
        elif gmm.covariance_type == 'diag':
            covariances = np.diag(gmm.covariances_[n][:2])
        elif gmm.covariance_type == 'spherical':
            covariances = np.eye(gmm.means_.shape[1]) * gmm.covariances_[n]
        v, w = np.linalg.eigh(covariances)
        u = w[0] / np.linalg.norm(w[0])
        angle = np.arctan2(u[1], u[0])
        angle = 180 * angle / np.pi  # convert to degrees
        v = 2. * np.sqrt(2.) * np.sqrt(v)
        ell = mpl.patches.Ellipse(gmm.means_[n, :2], v[0], v[1],
                                  180 + angle, color=color)
        ell.set_clip_box(ax.bbox)
        ell.set_alpha(0.5)
        ax.add_artist(ell)


if __name__ == '__main__':
    # Read depth images
    src = io.imread(os.path.join(image_folder, image_name_pattern.format("02")), as_grey=True)
    #src = io.imread("/home/def/Downloads/414685.jpg", as_grey=True)
    #src = io.imread("/home/def/Downloads/wrinkles-out-chiffon_99fe4d4c783282da.jpg", as_grey=True)
    io.imshow(src)

    # Set ROI
    roi_rect = ((84, 10) , (501, 480))
    roi = src[roi_rect[0][1]:roi_rect[1][1], roi_rect[0][0]:roi_rect[1][0]]
    io.imshow(roi)
    io.show()

    # Compute hessian -> curvatures -> shape index
    wrinkles = shape_index_filter(roi, lower_limit=-0.125, upper_limit=0.625, hessian_sigma=15)
    io.imshow(wrinkles)
    io.show()

    # Find connected regions and compute volume
    labels, n_labels = morphology.label(wrinkles, return_num=True)
    volumes = []
    for i in range(n_labels):
        volumes.append((i, np.sum(np.where(labels == i, labels, 0))))

    ordered_volumes = sorted(volumes, key=itemgetter(1))
    print(ordered_volumes)
    high_bumps = np.zeros_like(wrinkles) # High bumps
    high_bumps_labels = []
    for label, volume in ordered_volumes:
        if volume > 10000:
            #high_bumps = np.bitwise_or(high_bumps, np.where(labels==label, labels, 0))
            high_bumps = np.where(labels == label, roi, high_bumps)
            high_bumps_labels.append(label)

    io.imshow(high_bumps)
    io.show()

    # Note: Principal components are said to be computed, but they are never really used later
    # in the thesis, so I'm not computing them. If I were, that computation would go here.

    # Fit GMMs to wrinkles
    data = np.transpose(np.nonzero(high_bumps))
    n_classes = len(high_bumps_labels)

    clf = GaussianMixture(n_components=n_classes, covariance_type='full')
    clf.fit(data)

    # display predicted scores by the model as a contour plot
    x = np.linspace(roi_rect[0][0], roi_rect[1][0])
    y = np.linspace(roi_rect[0][1], roi_rect[1][1])
    X, Y = np.meshgrid(x, y)
    XX = np.array([X.ravel(), Y.ravel()]).T
    Z = -clf.score_samples(XX)
    Z = Z.reshape(X.shape)

    CS = plt.contour(X, Y, Z)
    CB = plt.colorbar(CS, shrink=0.8, extend='both')
    plt.scatter(data[:, 0], data[:, 1], .8)
    plt.show()

