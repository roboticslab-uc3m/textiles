import cv2
import numpy as np

from unfolding.perception.GarmentSegmentation import GarmentSegmentation

class GarmentDepthSegmentation(GarmentSegmentation):
    @staticmethod
    def background_subtraction(image):
        """
        Segments the garment from the background. This implementation takes a point cloud
        and uses RANSAC to segment it from the table
        :param image: Point Cloud filepath
        :return: Segmentation mask where white is garment and black is background
        """



        return filtered_mask_open
