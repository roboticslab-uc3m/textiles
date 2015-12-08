__author__ = 'def'

import numpy as np
import pylab
from skimage.filters.rank import median
from skimage.morphology import disk

if __name__ == '__main__':
    src = np.loadtxt('../pcl/build/depth_image.m')
    print np.unique(src).shape
    filtered_src = median(src, disk(5))
    pylab.figure(0)
    pylab.imshow(src)
    pylab.figure(1)
    pylab.imshow(filtered_src)
    pylab.show()