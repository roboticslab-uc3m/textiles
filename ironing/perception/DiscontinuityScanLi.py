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

def load_roi_from_file(filename):
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

def crop_roi(roi_rect, image):
    """
    Uses a ROI rectangle to crop a Region Of Interest of the image
    """
    return image[roi_rect[0][1]:roi_rect[1][1], roi_rect[0][0]:roi_rect[1][0]]

class DiscontinuityScanLi(object):
    image_wrinkles_name_pattern = "garment-{:02d}-image-{:02d}.ppm"
    image_reference_name_pattern = "garment-{:02d}-imageref-{:02d}.ppm"
    image_roi_name_pattern = "garment-{:02d}-roi.txt"

    def __init__(self):
        self.image_wrinkles_1 = None
        self.image_wrinkles_2 = None
        self.image_ref_1 = None
        self.image_ref_2 = None
        self.norm = None
        self.norm_1 = None
        self.norm_2 = None
        self.roi_rect = None

    def load_images(self, image_folder, image_id=0, use_roi=True):
        image_folder = os.path.abspath(os.path.expanduser(image_folder))

        self.image_wrinkles_1 = io.imread(os.path.join(image_folder, self.image_wrinkles_name_pattern.format(image_id, 1)), as_grey=True)
        self.image_wrinkles_2 = io.imread(os.path.join(image_folder, self.image_wrinkles_name_pattern.format(image_id, 2)), as_grey=True)

        self.image_ref_1 = io.imread(os.path.join(image_folder, self.image_reference_name_pattern.format(image_id, 1)), as_grey=True)
        self.image_ref_2 = io.imread(os.path.join(image_folder, self.image_reference_name_pattern.format(image_id, 2)), as_grey=True)

        if use_roi:
            self.roi_rect = load_roi_from_file(os.path.join(image_folder, self.image_roi_name_pattern.format(image_id)))

            self.image_wrinkles_1 = crop_roi(self.roi_rect, self.image_wrinkles_1)
            self.image_wrinkles_2 = crop_roi(self.roi_rect, self.image_wrinkles_2)
            self.image_ref_1 = crop_roi(self.roi_rect, self.image_ref_1)
            self.image_ref_2 = crop_roi(self.roi_rect, self.image_ref_2)


    def normalize_images(self):
        """
        Apply normalization method as described in Li's algorithm
        """
        self.norm_1 = self.image_wrinkles_1 / self.image_ref_1
        self.norm_2 = self.image_wrinkles_2 / self.image_ref_2
        self.norm = np.sqrt(np.power(self.norm_1,2)+np.power(self.norm_2,2))

    def compute_SIFT(self):
        # SIFT features
        # sift = cv2.xfeatures2d.SIFT_create()
        # kp = sift.detect(gray,None)
        #
        # img=cv2.drawKeypoints(gray,kp)
        # cv2.imwrite('sift_keypoints.jpg',img)
        pass

    def plot_input_images(self, colormap=plt.cm.viridis):
        """
        Plot all 4 input images in a single figure
        """
        f, ax = plt.subplots(2, 2)
        ax[0][0].imshow(self.image_wrinkles_1, cmap=colormap)
        ax[0][1].imshow(self.image_wrinkles_2, cmap=colormap)
        ax[1][0].imshow(self.image_ref_1, cmap=colormap)
        ax[1][1].imshow(self.image_ref_2, cmap=colormap)
        plt.show()

    def plot_normalized_images(self, colormap=plt.cm.viridis):
        f, ax = plt.subplots(1, 3)
        ax[0].imshow(self.norm_1, cmap=colormap)
        ax[1].imshow(self.norm_2, cmap=colormap)
        ax[2].imshow(self.norm , cmap=colormap)
        plt.show()


@begin.start(auto_convert=True)
def main(num_images: 'Number of images in image folder' = 0, display_results: 'Show feedback of the process' = False,
         generate_dataset: 'Generate dataset from input images' = False):

    image_folder = "~/Research/jResearch/2016-11-24-replicate-li/"
    
    discontinuity_scanner = DiscontinuityScanLi()

    for i in range(1, num_images+1): # For each sample image
        discontinuity_scanner.load_images(image_folder, image_id=i, use_roi=True)
        discontinuity_scanner.normalize_images()

        if display_results:
            discontinuity_scanner.plot_input_images()
            discontinuity_scanner.plot_normalized_images()

        if generate_dataset:
            image_output_name_pattern = "garment-{:02d}-out.png"
            image_folder = os.path.abspath(os.path.expanduser(image_folder))

            # Generate dataset for labeling
            colormap = plt.cm.viridis # or gray, inferno, etc
            io.imsave(os.path.join(image_folder, image_output_name_pattern.format(i)), colormap(normalize_array(discontinuity_scanner.norm)))
