# coding=utf-8

import os
from operator import itemgetter
from skimage import io
from skimage import feature
from skimage import morphology
import numpy as np

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


if __name__ == '__main__':
    # Read depth images
    src = io.imread(os.path.join(image_folder, image_name_pattern.format("03")), as_grey=True)
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
    n_bumps = 0 # Stores the number of high bumps detected
    for label, volume in ordered_volumes:
        if volume > 10000:
            high_bumps = np.bitwise_or(high_bumps, np.where(labels==label, labels, 0))
            n_bumps += 1

    io.imshow(high_bumps)
    io.show()

