import numpy as np
import matplotlib.pyplot as plt
from skimage.filters.rank import median
from skimage.morphology import disk

__author__ = 'def'

if __name__ == '__main__':
    image = np.loadtxt('../pcl/build/histogram_image.m')
    normalized_image = image/image.max()
    filtered_image = median(normalized_image, disk(3))

    fig, axes = plt.subplots(1, 2)
    axes[0].imshow(normalized_image, cmap=plt.cm.spectral)
    axes[1].imshow(filtered_image, cmap=plt.cm.hot)
    plt.show()