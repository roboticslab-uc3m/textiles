import cv2
import numpy as np
from segmentation import get_coloured_item
from matplotlib import pyplot as plt

__author__ = 'def'

def show_image(text, image):
    cv2.imshow(text, image)
    cv2.waitKey(200)

def process_mask(mask, kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))):
    """
        Input: binary mask, kernel size for closing
        Output: filtered contours contained in image
    """
    filtered_mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    contours, dummy = cv2.findContours(filtered_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    contours_filtered = []
    for contour in contours:
        if len(contour) > 20 and cv2.contourArea(contour) > 400:
            contours_filtered.append(contour)

    contours_filtered.sort(key=lambda x: cv2.contourArea(x))
    contours_filtered.reverse()

    return contours_filtered

def simplify_contour(contour):
    perimeter = cv2.arcLength(contour,True)
    approx = cv2.approxPolyDP(contour,0.12*perimeter,True)

    if len(approx) == 4 and cv2.isContourConvex(approx):
        return approx
    else:
        return None

def contour_center(contour):
    M = cv2.moments(contour)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    return (cx, cy)


def get_garment_contour(mask):
    # Get clothes contour:
    #ret, image_contours_src = cv2.threshold(image, 240, 255, cv2.THRESH_BINARY_INV)
    clothes_contours, dummy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    clothes_contour = clothes_contours[0]
    # Size filtering:
    maxArea = cv2.contourArea(clothes_contour)
    for contour in clothes_contours[1:]:
        currentArea = cv2.contourArea(contour)
        if currentArea > maxArea:
            maxArea = currentArea
            clothes_contour = contour

    # Simplify contour:
    perimeter = cv2.arcLength(clothes_contour, True)
    approx = cv2.approxPolyDP(clothes_contour, 0.010 * perimeter, True)
    return approx


def get_garment_main_lines(image):
    # Apply canny to find 'not cross' lines
    blur = cv2.GaussianBlur(image, (11, 11), 0)
    inverted = cv2.bitwise_not(blur)
    # show_image("gauss", inverted)
    edges = cv2.Canny(inverted, 80, 160, apertureSize=3)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    # show_image("canny", edges)
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=5)
    return edges


def get_highest_points(image, threshold=40):
    # Find highest zones
    ret, highest_zones = cv2.threshold(image, threshold, 255, cv2.THRESH_BINARY_INV)
    #highest_contours = process_mask(highest_zones)
    highest_contours, dummy = cv2.findContours(highest_zones.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    show_image("highest_zones", highest_zones)
    highest_points = []
    for contour in highest_contours:
        highest_points.append(contour_center(contour))

    return highest_points

def main():
    image_paths = ['./data/robe01_1_fold.ppm', './data/robe01_2_fold.ppm', './data/sweater02_1_fold.ppm', './data/sweater02_2_fold.ppm', './data/tshirt01_1_fold.ppm',
                   './data/tshirt01_2_fold.ppm','./data/polo01_1_fold.ppm', './data/polo01_2_fold.ppm', './data/dishcloth01_2_fold.ppm', './data/dishcloth01_1_fold.ppm']

    depth_maps = ['./data/robe01_1_fold.mat', './data/robe01_2_fold.mat', './data/sweater02_1_fold.mat', './data/sweater02_2_fold.mat', './data/tshirt01_1_fold.mat',
                   './data/tshirt01_2_fold.mat','./data/polo01_1_fold.mat', './data/polo01_2_fold.mat', './data/dishcloth01_2_fold.mat', './data/dishcloth01_1_fold.mat']

    for path_rgb, path_depth in zip(image_paths, depth_maps):
        # Load image
        image = cv2.imread(path_rgb)

        # Get mask
        mask = get_coloured_item(image)
        show_image("mask", mask)
        cv2.destroyAllWindows()

        # Obtain garment contour
        approx = get_garment_contour(mask)

        # Print clothes contour
        contour_show = image.copy()
        cv2.drawContours(contour_show, [approx], -1, (0, 0,255))
        for point in approx:
            cv2.circle(contour_show, tuple(point[0]), 3, (0, 0, 255), 2)
        show_image("contour", contour_show)

        # Load depth map
        depth_image = np.loadtxt(path_depth)
        masked_depth_image = np.where(mask[:,:,0]==255, depth_image.transpose(), 1000)
        # plt.figure(1)
        # plt.hist(masked_depth_image)

        # Normalize depth map
        scaled_depth_map = masked_depth_image.copy()
        min_value = masked_depth_image[np.unravel_index(np.where(masked_depth_image == 0, 1000,masked_depth_image).argmin(), masked_depth_image.shape)]
        max_value = masked_depth_image[np.unravel_index(np.where(masked_depth_image == 1000, 0,masked_depth_image).argmax(), masked_depth_image.shape)]
        range_value = max_value-min_value
        print "Depth image range: (%d, %d) l=%d" % (min_value, max_value, range_value)
        scaled_depth_map = np.where(scaled_depth_map != 1000, (scaled_depth_map - min_value) * (255/range_value), 255)
        # scaled_depth_map -= min_value
        # scaled_depth_map *= 255/range_value
        scaled_depth_map = scaled_depth_map.astype(np.uint8)
        # plt.figure(2)
        # plt.imshow(scaled_depth_map.astype(np.uint8))
        show_image("scaled", scaled_depth_map)
        # plt.show()

        # Get 'not crossing' lines
        edges = get_garment_main_lines(scaled_depth_map)
        show_image("canny2", edges)

        # Get highest_points
        try:
            highest_points = get_highest_points(scaled_depth_map,255/range_value*2)
        except:
            pass

        # Print clothes contour
        hp_show = image.copy()
        cv2.drawContours(hp_show, [approx], -1, (0, 0,255))
        for point in approx:
            cv2.circle(hp_show, tuple(point[0]), 3, (0, 0, 255), 2)
        for point in highest_points:
            cv2.circle(hp_show, tuple(point), 3, (255, 0, 0), 2)
        show_image("h point", hp_show)

        cv2.waitKey(-1)
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()