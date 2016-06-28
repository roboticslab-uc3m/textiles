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
        print "usage: pcl_plot_histogram [histogram_file.m]"
        #image_filenames = ["../pcl/build/histogram_image.m"]
        image_filenames = ["/home/def/Repositories/textiles/data/view_colored/histogram_pants3.m",
                           "/home/def/Repositories/textiles/data/view_colored/histogram_pants2.m"]

    for image_filename in image_filenames:
        try:
            image = np.loadtxt(image_filename)
        except IOError:
            print "Skipping " + image_filename
            continue
        normalized_image = image/image.max()
        filtered_image = median(normalized_image, disk(3))

        fig, axes = plt.subplots(1, 2)
        axes[0].imshow(normalized_image, cmap=plt.cm.hot)
        axes[1].imshow(filtered_image, cmap=plt.cm.hot)
    plt.show()