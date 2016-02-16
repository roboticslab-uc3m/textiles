__author__ = "def"

import cv2
import numpy as np

from GarmentSegmentation import GarmentSegmentation
from GarmentDepthMapAnalysis import GarmentDepthMapAnalysis
from GarmentPickAndPlacePoints import GarmentPickAndPlacePoints
from utils import load_data

if __name__ == "__main__":

    image_paths, depth_image_paths = load_data('../data/20150902')

    for path_rgb_image, path_depth_image in zip(image_paths, depth_image_paths):
        # Load input data
        image_src = cv2.imread(path_rgb_image)
        depth_image = np.loadtxt(path_depth_image)

        # Garment Segmentation Step
        mask = GarmentSegmentation.background_substraction()
        approx = GarmentSegmentation.calculate_approx_polygon()
