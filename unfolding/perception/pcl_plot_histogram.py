# -*- coding: utf-8 -*-
"""
pcl_plot_histogram allows visualization of z-histogram images generated with
HistogramImageCreator class
"""

import sys
import numpy as np
import matplotlib.pyplot as plt
from skimage.filters.rank import median
from skimage.morphology import disk

__author__ = 'def'

if __name__ == '__main__':
    image_filenames = sys.argv[1:]
    if not image_filenames:
        print("usage: pcl_plot_histogram [histogram_file.m]")
        # image_filenames = ["../pcl/build/histogram_image.m"]
        image_filenames = ["/home/def/Repositories/textiles/build/ironing/perception/image_mask.m",
                           "/home/def/Repositories/textiles/build/ironing/perception/wild_mean_image.m"]

    for image_filename in image_filenames:
        try:
            image = np.loadtxt(image_filename)
        except IOError:
            print( "Skipping " + image_filename)
            continue
        # normalized_image = image/image.max()
        # This normalizes images where 0 means background
        minimum = np.min(image[np.nonzero(image)])
        normalized_image = np.where( image != 0, (image - minimum) / (image.max()-minimum), 0)
        filtered_image = median(normalized_image, disk(3))

        fig, axes = plt.subplots(1, 2)
        axes[0].imshow(normalized_image, cmap=plt.cm.hot)
        axes[1].imshow(filtered_image, cmap=plt.cm.hot)

    plt.show()
