# -*- coding: utf-8 -*-
"""
Created on Wed Sep  2 11:42:35 2015

@author: smorante
"""
import matplotlib.pyplot as plt
import numpy as np
import cv2

from segmentation import get_coloured_item
from utils import load_data
from mpl_toolkits.mplot3d import Axes3D
from skimage.segmentation import  slic

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


def main():

    image_paths, depth_maps = load_data('./data/20150625_2')

    for path_rgb, path_depth in zip(image_paths, depth_maps):
        # Load image
        image = cv2.imread(path_rgb)
        print "Loaded rgb image, dimensions: " + str(image.shape)

        # Get mask
        mask = get_coloured_item(image)
#        show_image("mask", mask)
#        cv2.destroyAllWindows()

        # Obtain garment contour
        approx = get_garment_contour(mask)

        # Print clothes contour
        contour_show = image.copy()
        cv2.drawContours(contour_show, [approx], -1, (0, 0,255))
        for point in approx:
            cv2.circle(contour_show, tuple(point[0]), 3, (0, 0, 255), 2)
#        show_image("contour", contour_show)

        # Load depth map
        depth_image = np.loadtxt(path_depth)
        masked_depth_image = np.where(mask[:,:,0]==255, depth_image.transpose(), 1000)
        print "Loaded depth image, dimensions: " + str(depth_image.shape)


        # Normalize depth map
        scaled_depth_map = masked_depth_image.copy()
        min_value = masked_depth_image[np.unravel_index(np.where(masked_depth_image == 0, 1000,masked_depth_image).argmin(), masked_depth_image.shape)]
        max_value = masked_depth_image[np.unravel_index(np.where(masked_depth_image == 1000, 0,masked_depth_image).argmax(), masked_depth_image.shape)]
        range_value = max_value-min_value
        print "Scaled depth image dimensions: " + str(scaled_depth_map.shape)
        print "Depth image range: (%d, %d) delta=%d" % (min_value, max_value, range_value)
        scaled_depth_map = np.where(scaled_depth_map != 1000, (scaled_depth_map - min_value) * (255/range_value), 255)
        scaled_depth_map = scaled_depth_map.astype(np.uint8)
#        plt.figure(2)
#        plt.imshow(scaled_depth_map.astype(np.uint8))
#        cv2.imshow("scaled", scaled_depth_map)

##########       

#        X = np.arange(0, scaled_depth_map.shape[1], 1)
#        Y = np.arange(0, scaled_depth_map.shape[0], 1)
#        X, Y = np.meshgrid(X, Y)
#        plt.scatter(X,Y,scaled_depth_map)
#        plt.show()

        pseudo_3d_data = []
        
        for i in range(scaled_depth_map.shape[0]):
            for j in range(scaled_depth_map.shape[1]):
              #  print i, j, scaled_depth_map[i,j]
                pseudo_3d_data.append([i, j, scaled_depth_map[i,j]])


        import Superpixels
        segments_slic = slic(np.array(pseudo_3d_data).astype(float), n_segments=250, compactness=10, sigma=1, min_size_factor=200)
        avg = Superpixels.get_average_slic(scaled_depth_map, segments_slic)


if __name__ == "__main__":
    main()