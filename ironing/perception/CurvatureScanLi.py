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
from common.perception.roi import crop_roi

"""
Curvature Scan Li
---------------------------------------------------------------------
Curvature scan as described in Yinxiao Li's Thesis and in article:
 * Y. Li, X. Hu, D. Xu, Y. Yue, E. Grinspun, and P. Allen, “Multi-
Sensor Surface Analysis for Robotic Ironing,” in IEEE International
Conference on Robotics and Automation (ICRA), Stockholm, 2016.

Finds height bumps on garments based on depth data.
"""

class CurvatureScanLi(object):
    image_name_pattern = "garment-{}-depth.ppm"

    def __init__(self):
        raise NotImplementedError

    def load_images(self, image_folder, image_id=0, use_roi=True):
        raise NotImplementedError

    @staticmethod
    def shape_index(img, lower_limit, upper_limit, hessian_sigma=1):
        raise NotImplementedError

    @staticmethod
    def shape_index_filter(img, lower_limit, upper_limit, hessian_sigma=1):
        raise NotImplementedError

    @staticmethod
    def compute_normalized_volume(img, mask):
        raise NotImplementedError

    @staticmethod
    def normalized_volume_filter(img, mask, threshold):
        raise NotImplementedError

    def fit_GMMs(self):
        raise NotImplementedError

    def run(self):
        raise NotImplementedError


def depthMap_2_heightMap(depth_map):
    """
    Gets a Depth Map, returns a Height Map
    """
    max_val = depth_map.max()
    return max_val - depth_map

def shape_index_filter(img, lower_limit, upper_limit, hessian_sigma=1):
    """
    Filter curvature regions in a depth image using the shape index as criteria.

    The shape index is computed using the eigenvectors of the Hessian matrix.

    :param img: Depth image to filter regions from
    :param lower_limit: Lower threshold of shape index value
    :param upper_limit: Upper threshold of shape index value
    :param hessian_sigma: Sigma used when computing the Hessian matrix
    :return:
    """
    Hxx, Hxy, Hyy = feature.hessian_matrix(img, sigma=hessian_sigma, mode='nearest')
    k1, k2 = feature.hessian_matrix_eigvals(Hxx, Hxy, Hyy)
    shape_index = 2 / np.pi * np.arctan((k1 + k2) / (k1 - k2))
    io.imshow(shape_index)
    io.show()
    wrinkles_low = np.where(lower_limit <= shape_index, 1, 0)
    wrinkles_upper = np.where(shape_index < upper_limit, 1, 0)
    return np.bitwise_and(wrinkles_low, wrinkles_upper)


def plot_gmm(gmm, ax):
    """
    Plot a Gaussian Mixture Model (GMM) in a existing figure
    :param gmm: Gaussian Mixture Model
    :param ax: Existing figure
    """
    colors = ['navy', 'turquoise', 'darkorange']

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
    image_folder = "~/Research/jresearch/2016-10-10-replicate-li/"
    image_folder = os.path.abspath(os.path.expanduser(image_folder))
    image_name_pattern = "garment-{}-depth.ppm"

    # Read depth images
    src = io.imread(os.path.join(image_folder, image_name_pattern.format("02")), as_grey=True)
    src = depthMap_2_heightMap(src)
    io.imshow(src)

    # Set ROI
    roi_rect = ((84, 10), (501, 480))
    roi = crop_roi(roi_rect, src)
    io.imshow(roi)
    io.show()

    # Filter according to shape index
    # Note: it uses eigenvectors of the Hessian to compute shape index internally
    wrinkles = shape_index_filter(roi, lower_limit=-0.125, upper_limit=0.625, hessian_sigma=15)
    io.imshow(wrinkles)
    io.show()

    # Find connected regions and compute volume
    labels, n_labels = morphology.label(wrinkles, background=0, return_num=True)
    volumes = []
    for i in range(1, n_labels):
        max_h = np.where(labels == i, roi, 0).max()
        min_h = np.min(np.where(labels == i, roi, max_h))
        volumes.append((i, np.sum(np.where(labels == i, (roi - min_h) / (max_h - min_h), 0))))

    high_bumps = np.zeros_like(roi) # High bumps
    high_bumps_labels = []
    for label, volume in volumes:
        if volume > 1000:
            #high_bumps = np.bitwise_or(high_bumps, np.where(labels==label, labels, 0))
            high_bumps = np.where(labels==label, roi, high_bumps)
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

