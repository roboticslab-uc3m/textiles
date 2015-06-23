import cv2
import numpy as np
from matplotlib import pyplot as plt

__author__ = 'def'


def get_coloured_item(image):
    # Convert to HSV color space
    image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Threshold value and saturation (using Otsu for threshold selection)
    ret, mask_s = cv2.threshold(image_hsv[:, :, 1], 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    ret, mask_v = cv2.threshold(image_hsv[:, :, 2], 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    # mask = cv2.inRange(image_hsv, (0, 0, 150), (180, 100, 255))
    mask = cv2.bitwise_and(mask_s, mask_v)

    # Filter result using morphological operations (closing)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    filtered_mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=3)

    # Get contours, filter them and fill them to get the final mask:
    contours, dummy = cv2.findContours(filtered_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_filtered = []
    for contour in contours:
        if len(contour) > 20 and cv2.contourArea(contour) > 150:
            contours_filtered.append(contour)
    contours_filtered.sort(key=lambda x: cv2.contourArea(x))
    contours_filtered.reverse()

    final_mask = np.zeros([image.shape[0], image.shape[1], 1], np.uint8)
    cv2.drawContours(final_mask, contours_filtered, 0, 255, -1)

    print final_mask.shape
    return final_mask

def main():
    image_paths = ['./data/robe01_1_fold.ppm', './data/robe01_2_fold.ppm', './data/sweater02_1_fold.ppm', './data/sweater02_2_fold.ppm', './data/tshirt01_1_fold.ppm',
                   './data/tshirt01_2_fold.ppm','./data/polo01_1_fold.ppm', './data/polo01_2_fold.ppm', './data/dishcloth01_2_fold.ppm', './data/dishcloth01_1_fold.ppm']

    for path in image_paths:
        image = cv2.imread(path)

        mask = get_coloured_item(image)
        cv2.imshow("mask", mask)
        cv2.waitKey(500)
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
