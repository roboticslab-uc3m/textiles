__author__ = 'def'

from scipy import ndimage as ndi
from skimage.morphology import watershed, disk
from skimage.filters import rank
from skimage.util import img_as_ubyte
from skimage.restoration import denoise_tv_chambolle
from skimage import exposure

import matplotlib.pyplot as plt

class GarmentAnalysis:
    def __init__(self):
        self.n_patches = 0

    def compute(self, src):
        image = img_as_ubyte(src)

        # denoise image
        denoised = denoise_tv_chambolle(image, weight=0.05)
        denoised_equalize= exposure.equalize_hist(denoised)

        # find continuous region (low gradient) --> markers
        markers = rank.gradient(denoised_equalize, disk(5)) < 10
        markers = ndi.label(markers)[0]

        # local gradient
        gradient = rank.gradient(denoised, disk(2))

        # labels
        labels = watershed(gradient, markers)

        # display results
        fig, axes = plt.subplots(2,3)
        axes[0, 0].imshow(image)#, cmap=plt.cm.spectral, interpolation='nearest')
        axes[0, 1].imshow(denoised, cmap=plt.cm.spectral, interpolation='nearest')
        axes[0, 2].imshow(markers, cmap=plt.cm.spectral, interpolation='nearest')
        axes[1, 0].imshow(gradient, cmap=plt.cm.spectral, interpolation='nearest')
        axes[1, 1].imshow(labels, cmap=plt.cm.spectral, interpolation='nearest', alpha=.7)
        plt.show()


if __name__ == '__main__':
    import numpy as np

    src = np.loadtxt('../pcl/build/cube.m')
    print "Loaded point cloud data with %d unique values" % np.unique(src).shape[0]
    filtered_src = rank.median(src, disk(5))
    plt.figure(0)
    plt.imshow(src)
    plt.figure(1)
    plt.imshow(filtered_src)
    plt.show()

    # ga = GarmentAnalysis.GarmentAnalysis()
    # ga.compute(filtered_src)
    # print "Found %d pathes" % ga.n_patches
