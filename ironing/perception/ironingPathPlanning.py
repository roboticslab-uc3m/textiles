# -*- coding: utf-8 -*-
"""
ironingPathPlanning computes ironing paths from wrinkle data
"""

import sys
import numpy as np
import matplotlib.pyplot as plt
from skimage.filters.rank import median
from skimage.morphology import disk
from skimage import img_as_ubyte
import cv2
from skimage.filters import frangi, hessian

image_filename = "/home/def/Repositories/textiles/build/ironing/perception/wild_mean_image.m"
mask_filename = "/home/def/Repositories/textiles/build/ironing/perception/image_mask.m"

__author__ = 'def'

if __name__ == '__main__':

    # Load data from files
    image = np.loadtxt(image_filename)
    mask = np.loadtxt(mask_filename)
    mask = img_as_ubyte(mask)

    # Normalize (?)
    minimum = np.min(image[np.nonzero(image)])
    normalized_image = np.where( image != 0, (image - minimum) / (image.max()-minimum), 0)
    filtered_image = median(normalized_image, disk(3))

    # Step #1: Identify garment border
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    garment_contour = max(contours, key=cv2.contourArea)

    # Display the image and plot all contours found
    fig, ax = plt.subplots()
    ax.imshow(normalized_image, interpolation='nearest', cmap=plt.cm.RdGy)

    points = [tuple(point[0]) for point in garment_contour]
    # Plot lines
    for (start_x, start_y), (end_x, end_y) in zip(points, points[1:]+points[0:1]):
        plt.plot( (start_x, end_x), (start_y, end_y), 'r-', linewidth=2.0, alpha=0.7 )
    # Plot points
    for x, y in points:
        plt.plot(x, y, 'ro', alpha=0.7)

    ax.axis('image')
    ax.set_xticks([])
    ax.set_yticks([])
    plt.show()

    # Step #2: Identify wrinkle
    fig, ax = plt.subplots(ncols=3, subplot_kw={'adjustable': 'box-forced'})

    ax[0].imshow(image, cmap=plt.cm.gray)
    ax[0].set_title('Original image')

    ax[1].imshow(frangi(image), cmap=plt.cm.gray)
    ax[1].set_title('Frangi filter result')

    ax[2].imshow(hessian(image), cmap=plt.cm.gray)
    ax[2].set_title('Hybrid Hessian filter result')

    for a in ax:
        a.axis('off')

    plt.tight_layout()
    plt.show()