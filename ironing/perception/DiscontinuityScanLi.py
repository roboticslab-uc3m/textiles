# coding=utf-8

import os
from skimage import io
import numpy as np
import matplotlib.pyplot as plt
import begin
from common.math import normalize_array

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

image_folder = "~/Research/jResearch/2016-11-24-replicate-li/"
image_folder = os.path.abspath(os.path.expanduser(image_folder))
image_wrinkles_name_pattern = "garment-{:02d}-image-{:02d}.ppm"
image_reference_name_pattern = "garment-{:02d}-imageref-{:02d}.ppm"
image_roi_name_pattern = "garment-{:02d}-roi.txt"
image_output_name_pattern = "garment-{:02d}-out.png"

def load_foi_from_file(filename):
    """
    Loads region of interest from a file. The file format is the following:
    start_x start_y
    end_x end_y
    :param filename: Path to the file
    :return: a region of interest described as two points -> (start_x, start_y), (end_x, end_y)
    """

    with open(filename, 'r') as f:
        lines = f.readlines()
        (start_x, start_y), (end_x, end_y) = [[int(p) for p in line.split(' ')] for line in lines]

    return (start_x, start_y), (end_x, end_y)

@begin.start(auto_convert=True)
def main(num_images: "Number of images in image folder"=0, generate_dataset: "Generate the dataset for annotations" = False,
            display_results: "Show feedback of the process" = True):
    for i in range(1, num_images+1): # For each sample image
        # Load source images
        image_wrinkles_1 = io.imread(os.path.join(image_folder, image_wrinkles_name_pattern.format(i, 1)), as_grey=True)
        image_wrinkles_2 = io.imread(os.path.join(image_folder, image_wrinkles_name_pattern.format(i, 2)), as_grey=True)

        image_ref_1 = io.imread(os.path.join(image_folder, image_reference_name_pattern.format(i, 1)), as_grey=True)
        image_ref_2 = io.imread(os.path.join(image_folder, image_reference_name_pattern.format(i, 2)), as_grey=True)

        if display_results:
            f, ax = plt.subplots(2, 2)
            ax[0][0].imshow(image_wrinkles_1, cmap=plt.cm.viridis)
            ax[0][1].imshow(image_wrinkles_2, cmap=plt.cm.viridis)
            ax[1][0].imshow(image_ref_1, cmap=plt.cm.viridis)
            ax[1][1].imshow(image_ref_2, cmap=plt.cm.viridis)
            plt.show()

        # Normalize images
        norm_1 = image_wrinkles_1 / image_ref_1
        norm_2 = image_wrinkles_2 / image_ref_2
        norm = np.sqrt(np.power(norm_1,2)+np.power(norm_2,2))

        # Set ROIs
        roi_rect = load_foi_from_file(os.path.join(image_folder, image_roi_name_pattern.format(i)))
        roi1 = norm_1[roi_rect[0][1]:roi_rect[1][1], roi_rect[0][0]:roi_rect[1][0]]
        roi2 = norm_2[roi_rect[0][1]:roi_rect[1][1], roi_rect[0][0]:roi_rect[1][0]]
        roi = norm[roi_rect[0][1]:roi_rect[1][1], roi_rect[0][0]:roi_rect[1][0]]

        if display_results:
            f, ax = plt.subplots(1, 3)
            # normalized_image = (roi1 - np.min(roi1)) / (np.max(roi1)-np.min(roi1))
            ax[0].imshow(roi1, cmap=plt.cm.viridis)
            ax[1].imshow(roi2, cmap=plt.cm.viridis)
            ax[2].imshow(roi , cmap=plt.cm.viridis)
            plt.show()

        if generate_dataset:
            # Generate dataset for annotations
            colormap = plt.cm.inferno # or viridis, gray
            io.imsave(os.path.join(image_folder, image_output_name_pattern.format(i)), colormap(normalize_array(roi)))

        # SIFT features
        # sift = cv2.xfeatures2d.SIFT_create()
        # kp = sift.detect(gray,None)
        #
        # img=cv2.drawKeypoints(gray,kp)
        # cv2.imwrite('sift_keypoints.jpg',img)