import cv2
import numpy as np


class GarmentSegmentation:
    @staticmethod
    def background_subtraction(image):
        """
        Segments the garment from the background. This implementation returns colorful and dark
        objects as garments, and white and clear objects as background.
        :param image: Input image
        :return: Segmentation mask where white is garment and black is background
        """
        # Convert to HSV color space
        image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Threshold value and saturation (using Otsu for threshold selection)
        blur_s = cv2.GaussianBlur(image_hsv[:, :, 1], (5, 5), 0)
        ret, mask_s = cv2.threshold(blur_s, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        # cv2.imshow("----", mask_s)

        blur_v = cv2.GaussianBlur(image_hsv[:, :, 2], (5, 5), 0)
        ret, mask_v = cv2.threshold(blur_v, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        mask = cv2.bitwise_and(mask_s, mask_v)

        # Filter result using morphological operations (closing)
        kernel = np.ones((5, 5), np.uint8)
        filtered_mask_close = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=5)
        filtered_mask_open = cv2.morphologyEx(filtered_mask_close, cv2.MORPH_OPEN, kernel, iterations=8)

        return filtered_mask_open

    @staticmethod
    def compute_approximated_polygon(mask):
        """
        Calculate the approximated polygon that describes the garment
        :param mask: Segmentation mask where white is garment and black is background
        :return: Garment Approximated Polygon (as a vector of 2D points)
        """
        # Get clothes outline with largest area
        _, garment_outlines, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        largest_outline = max(garment_outlines, key=cv2.contourArea)

        # Simplify outline to approximated polygon:
        perimeter = cv2.arcLength(largest_outline, True)
        approximated_polygon = cv2.approxPolyDP(largest_outline, 0.010 * perimeter, True)
        return approximated_polygon
