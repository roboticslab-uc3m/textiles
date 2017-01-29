# coding=utf-8

from common.perception.roi import load_roi_from_file, crop_roi
from common.math import normalize

import os
from skimage import io
from skimage import feature
from skimage import morphology
import numpy as np
from sklearn.mixture import GaussianMixture
import matplotlib.pyplot as plt
import logging
logger = logging.getLogger(__name__)

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
    image_name_pattern = "garment-{:02d}-depth.ppm"
    image_roi_name_pattern = "garment-{:02d}-roi.txt"

    def __init__(self):
        self.height_map = None
        self.roi_rect = None
        self.wrinkles = None
        self.high_bumps = None
        self.high_bumps_labels = None
        self.gmm = None

    def load_images(self, image_folder, image_id=0, use_roi=True):
        image_folder = os.path.abspath(os.path.expanduser(image_folder))

        # Read depth image
        src = io.imread(os.path.join(image_folder, self.image_name_pattern.format(image_id)), as_grey=True)
        self.height_map = depthMap_2_heightMap(src)

        if use_roi:
            try:
                self.roi_rect = load_roi_from_file(os.path.join(image_folder, self.image_roi_name_pattern.format(image_id)))
                self.height_map = crop_roi(self.roi_rect, self.height_map)
            except FileNotFoundError:
                logger.warning("Could not load ROI, file does not exist")

    @staticmethod
    def shape_index(img, hessian_sigma=1, debug=False):
        """
        Compute the shape index of a given image. The shape index is computed using
        the eigenvectors of the Hessian matrix.

        The shape index is described in the following work:
        Koenderink, Jan J., and Andrea J. van Doorn. "Surface shape and curvature scales."
        Image and vision computing 10.8 (1992): 557-564.

        :param img: Input image
        :param hessian_sigma: Sigma used when computing the Hessian matrix
        :return: image containing the value of the shape index for each input pixel
        """
        Hxx, Hxy, Hyy = feature.hessian_matrix(img, sigma=hessian_sigma, mode='nearest')
        k1, k2 = feature.hessian_matrix_eigvals(Hxx, Hxy, Hyy)
        shape_index = 2 / np.pi * np.arctan((k1 + k2) / (k1 - k2))

        if debug:
            io.imshow(shape_index)
            io.show()

        return shape_index

    @staticmethod
    def shape_index_filter(img, lower_limit, upper_limit, hessian_sigma=1):
        """
        Filter curvature regions in a depth image using the shape index as criteria.

        The shape index is computed using the eigenvectors of the Hessian matrix.

        :param img: Depth image to filter regions from
        :param lower_limit: Lower threshold of shape index value
        :param upper_limit: Upper threshold of shape index value
        :param hessian_sigma: Sigma used when computing the Hessian matrix
        :return: mask representing regions with curvature between limits
        """
        shape_index = CurvatureScanLi.shape_index(img, hessian_sigma)
        wrinkles_low = np.where(lower_limit <= shape_index, 1, 0)
        wrinkles_upper = np.where(shape_index < upper_limit, 1, 0)
        return np.bitwise_and(wrinkles_low, wrinkles_upper)

    @staticmethod
    def compute_normalized_volume(img, mask):
        max_h = np.where(mask == 1, img, 0).max()
        min_h = np.min(np.where(mask == 1, img, max_h))
        return np.sum(np.where(mask == 1, (img - min_h) / (max_h - min_h), 0))

    def normalized_volume_filter(self, threshold):
        # Find connected regions and compute volume
        labels, n_labels = morphology.label(self.wrinkles, background=0, return_num=True)
        volumes = [ (i, self.compute_normalized_volume(self.height_map, np.where(labels==i, 1, 0)))
                    for i in range(1, n_labels)]

        self.high_bumps = np.zeros_like(self.height_map) # High bumps
        self.high_bumps_labels = []
        for label, volume in volumes:
            if volume > 1000:
                # high_bumps = np.bitwise_or(high_bumps, np.where(labels==label, labels, 0))
                self.high_bumps = np.where(labels==label, self.height_map, self.high_bumps)
                self.high_bumps_labels.append(label)

    def fit_GMMs(self, debug):
        """
        Fit wrinkles to a Gaussian Mixture Model
        """
        data = np.transpose(np.nonzero(self.high_bumps))
        n_classes = len(self.high_bumps_labels)

        self.gmm = GaussianMixture(n_components=n_classes, covariance_type='full')
        self.gmm.fit(data)

        # Generate probability image from GMM
        x = np.linspace(self.roi_rect[0][0], self.roi_rect[1][0])
        y = np.linspace(self.roi_rect[0][1], self.roi_rect[1][1])
        X, Y = np.meshgrid(x, y)
        XX = np.array([X.ravel(), Y.ravel()]).T
        Z = -self.gmm.score_samples(XX)
        Z = Z.reshape(X.shape)
        if debug:
            # display predicted scores by the model as a contour plot
            CS = plt.contour(X, Y, Z)
            CB = plt.colorbar(CS, shrink=0.8, extend='both')
            plt.scatter(data[:, 0], data[:, 1], .8)
            plt.show()

        return np.transpose(1-normalize(Z))

    def run(self, debug=False):
        # Filter according to shape index
        # Note: it uses eigenvectors of the Hessian to compute shape index internally
        self.wrinkles = self.shape_index_filter(self.height_map, lower_limit=-0.125, upper_limit=0.625, hessian_sigma=15)

        if debug:
            io.imshow(self.wrinkles)
            io.show()

        # Find connected regions and filter according to normalized volume
        self.normalized_volume_filter(threshold=1000)

        # Note: Principal components are said to be computed, but they are never really used later
        # in the thesis, so I'm not computing them. If I were, that computation would go here.

        # Fit GMMs to wrinkles
        self.fit_GMMs(debug)


def depthMap_2_heightMap(depth_map):
    """
    Gets a Depth Map, returns a Height Map
    """
    max_val = depth_map.max()
    return max_val - depth_map

if __name__ == '__main__':
    image_folder = "~/Research/jResearch/2016-10-10-replicate-li/"

    # Configure logger
    logging.basicConfig(level=logging.DEBUG)
    for handler in logging.root.handlers:
        handler.addFilter(logging.Filter(__name__))

    # Load images and process them
    curvature_scanner = CurvatureScanLi()
    curvature_scanner.load_images(image_folder, image_id=2, use_roi=True)
    curvature_scanner.run(debug=True)



