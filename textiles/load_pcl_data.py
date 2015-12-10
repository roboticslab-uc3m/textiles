__author__ = 'def'

import numpy as np
import pylab
from skimage.filters.rank import median
from skimage.morphology import disk

import GarmentAnalysis

def surface_plot(data):
    from mpl_toolkits.mplot3d import Axes3D
    import matplotlib.pyplot as plt

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    X = data[:,0]
    Y = data[:,1]
    # X, Y = np.meshgrid(X, Y)
    Z = data[:,2]
    G = data[:,4]
    N = G/G.max()  # normalize 0..1
    ax.xaxis.set_ticks(np.arange(-1, 1, 0.1))
    ax.yaxis.set_ticks(np.arange(-1, 1, 0.1))
    ax.set_autoscalex_on(False)
    ax.set_autoscaley_on(False)
    ax.scatter(X,Y,Z, c=N, cmap=plt.cm.spectral)

    # ax.plot(X, Y, Z, 'o', cmap=cm.jet(N))
    # surf = ax.plot_surface(
    #    X, Y, Z, rstride=1, cstride=1,
    #    facecolors=cm.jet(N),
    #    linewidth=0, antialiased=False, shade=False)
    plt.show()

if __name__ == '__main__':
    src = np.loadtxt('../pcl/build/depth_image2.m')
    print np.unique(src).shape
    filtered_src = median(src, disk(5))
    pylab.figure(0)
    pylab.imshow(src)
    pylab.figure(1)
    pylab.imshow(filtered_src)
    # pylab.show()

    # ga = GarmentAnalysis.GarmentAnalysis()
    # ga.compute(filtered_src)
    # print "Found %d pathes" % ga.n_patches

    data = np.loadtxt('../pcl/build/rsd_data.m')
    surface_plot(data)
