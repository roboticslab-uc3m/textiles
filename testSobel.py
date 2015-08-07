import numpy as np
import matplotlib.pyplot as plt

from skimage.filters import roberts, sobel, scharr
from skimage import io

import glob, os

images = glob.glob(os.path.join('./data/20150625_2', '*.png'))

for name in images:
    image = io.imread(name)
    edge_roberts = roberts(image)
    edge_sobel = sobel(image)

    thres =  edge_roberts >= 0.09
    thres2 = edge_sobel >= 0.09

    fig, (ax0, ax1) = plt.subplots(ncols=2, nrows=2)

    ax0[0].imshow(edge_roberts, cmap=plt.cm.gray)
    ax0[0].set_title('Roberts Edge Detection')
    ax0[0].axis('off')

    ax0[1].imshow(edge_sobel, cmap=plt.cm.gray)
    ax0[1].set_title('Sobel Edge Detection')
    ax0[1].axis('off')

    ax1[0].imshow(thres, cmap=plt.cm.gray)
    ax1[0].set_title('Roberts Edge Thresholded')
    ax1[0].axis('off')

    ax1[1].imshow(thres2, cmap=plt.cm.gray)
    ax1[1].set_title('Sobel Edge Thresholded')
    ax1[1].axis('off')

    plt.tight_layout()
    plt.show()