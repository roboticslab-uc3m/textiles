from __future__ import print_function

import matplotlib.pyplot as plt
import numpy as np

from skimage.data import lena
from skimage.segmentation import felzenszwalb, slic, quickshift
from skimage.segmentation import mark_boundaries
from skimage.util import img_as_float
from skimage import io
from skimage.color import gray2rgb
import glob, os

from Superpixels import *

images = glob.glob(os.path.join('./data/20150625_2', '*.png'))

for name in images:
    img_src = io.imread(name)
    img = gray2rgb(img_src)
    img = img_as_float(img[::2, ::2])

    segments_fz = felzenszwalb(img, scale=100, sigma=0.5, min_size=50)
    segments_slic = slic(img, n_segments=250, compactness=10, sigma=1, min_size_factor=200)
    segments_quick = quickshift(img, kernel_size=3, max_dist=6, ratio=0.5)

    print("Felzenszwalb's number of segments: %d" % len(np.unique(segments_fz)))
    print("Slic number of segments: %d" % len(np.unique(segments_slic)))
    print("Quickshift number of segments: %d" % len(np.unique(segments_quick)))


    fig, ax = plt.subplots(2, 2)
    fig.set_size_inches(8, 3, forward=True)
    fig.subplots_adjust(0.05, 0.05, 0.95, 0.95, 0.05, 0.05)

    ax[0][0].imshow(img_src[::2, ::2], cmap=plt.cm.gray)
    ax[0][0].set_title("SRC")
    ax[0][1].imshow(mark_boundaries(img, segments_slic))
    ax[0][1].set_title("SLIC")


    for b in ax:
        for a in b:
            a.set_xticks(())
            a.set_yticks(())

    avg = get_average_slic(img_src[::2, ::2], segments_slic)

    ax[1][0].imshow(avg, cmap=plt.cm.gray)
    ax[1][0].set_title("AVG")
    x, y = line_sampling_points([50, 10], [140, 98], 2)
    print([(i, j) for i,j in zip(x,y)])
    ax[1][0].plot(x, y, 'b.')

    highest_patch = get_highest_superpixel(avg)
    ax[1][1].imshow(highest_patch, cmap=plt.cm.gray)
    centroid = get_centroid(highest_patch)
    ax[1][1].plot(centroid[1], centroid[0], 'r+' )
    ax[1][1].set_title("HIGH")

    plt.figure()
    fn = line_sampling(avg, [50, 10], [140, 98], 2)
    fn2 = line_sampling(img_src[::2, ::2], [50, 10], [140, 98], 2)
    plt.plot(fn, 'b-', fn2, 'r-')
    plt.show()