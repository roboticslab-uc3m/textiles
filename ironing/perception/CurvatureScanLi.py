# coding=utf-8

import os
from skimage import io
from skimage import feature
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

    # Compute hessian
    for i in range(1, 11):
        Hxx, Hxy, Hyy = feature.hessian_matrix(roi, sigma=i)
        k1, k2 = feature.hessian_matrix_eigvals(Hxx, Hxy, Hyy)
        shape_index = 2/np.pi*np.arctan((k1+k2)/(k1-k2))
        lower_limit, upper_limit = -0.125, 0.625
        wrinkles_low = np.where(lower_limit <= shape_index, 1, 0)
        wrinkles_upper = np.where(shape_index < upper_limit, 1, 0)
        wrinkles = np.bitwise_and(wrinkles_low, wrinkles_upper)
        # wrinkles  = np.where(np.isclose(shape_index, np.zeros_like(shape_index), atol=0.1), 1, 0)
        io.imshow(wrinkles)

        io.show()