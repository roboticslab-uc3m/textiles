# coding=utf-8

import os
from skimage import io
import numpy as np
import matplotlib.pyplot as plt
import cv2

"""
Discontinuity Scan Li
---------------------------------------------------------------------
Discontinuity scan as described in Yinxiao Li's Thesis and in article:
 * Y. Li, X. Hu, D. Xu, Y. Yue, E. Grinspun, and P. Allen, “Multi-
Sensor Surface Analysis for Robotic Ironing,” in IEEE International
Conference on Robotics and Automation (ICRA), Stockholm, 2016.

Finds wrinkles on garments based on rgb images taken with two different
perpendicular illumination sources.
"""

image_folder = "~/Research/jresearch/2016-11-24-replicate-li/"
image_folder = os.path.abspath(os.path.expanduser(image_folder))
image_wrinkles_name_pattern = "garment-{:02d}-image-{:02d}.ppm"
image_reference_name_pattern = "garment-{:02d}-imageref-{:02d}.ppm"


if __name__ == '__main__':
    for i in range(1, 5): # For each sample image
        # Load source images
        image_wrinkles_1 = io.imread(os.path.join(image_folder, image_wrinkles_name_pattern.format(i, 1)), as_grey=True)
        image_wrinkles_2 = io.imread(os.path.join(image_folder, image_wrinkles_name_pattern.format(i, 2)), as_grey=True)

        image_ref_1 = io.imread(os.path.join(image_folder, image_reference_name_pattern.format(i, 1)), as_grey=True)
        image_ref_2 = io.imread(os.path.join(image_folder, image_reference_name_pattern.format(i, 2)), as_grey=True)

        f, ax = plt.subplots(2, 2)
        ax[0][0].imshow(image_wrinkles_1, cmap=plt.cm.viridis)
        ax[0][1].imshow(image_wrinkles_2, cmap=plt.cm.viridis)
        ax[1][0].imshow(image_ref_1, cmap=plt.cm.viridis)
        ax[1][1].imshow(image_ref_2, cmap=plt.cm.viridis)
        plt.show()

        # Normalize images
        norm_1 = image_wrinkles_1 / image_ref_1
        #norm_1 = np.nan_to_num(np.where(np.isinf(norm_1), 1, norm_1))
        #norm_1 = np.where(norm_1 <= 1, norm_1, 0)

        norm_2 = image_wrinkles_2 / image_ref_2
        #norm_2 = np.nan_to_num(np.where(np.isinf(norm_2), 1, norm_2))
        #norm_2 = np.where(norm_2 <= 1, norm_2, 0)
        norm = np.sqrt(np.power(norm_1,2)+np.power(norm_2,2))

        # Set ROIs
        roi_rect = ((140, 100), (430, 240))
        #roi_rect = ((84, 10) , (501, 480))
        roi1 = norm_1[roi_rect[0][1]:roi_rect[1][1], roi_rect[0][0]:roi_rect[1][0]]
        roi2 = norm_2[roi_rect[0][1]:roi_rect[1][1], roi_rect[0][0]:roi_rect[1][0]]
        roi = norm[roi_rect[0][1]:roi_rect[1][1], roi_rect[0][0]:roi_rect[1][0]]
        #roi1 = norm_1
        #roi2 = norm_2
        #roi = norm

        f, ax = plt.subplots(1, 3)
        # normalized_image = (roi1 - np.min(roi1)) / (np.max(roi1)-np.min(roi1))
        ax[0].imshow(roi1, cmap=plt.cm.viridis)
        ax[1].imshow(roi2, cmap=plt.cm.viridis)
        ax[2].imshow(roi , cmap=plt.cm.viridis)
        plt.show()

        # SIFT features
        # sift = cv2.xfeatures2d.SIFT_create()
        # kp = sift.detect(gray,None)
        #
        # img=cv2.drawKeypoints(gray,kp)
        # cv2.imwrite('sift_keypoints.jpg',img)