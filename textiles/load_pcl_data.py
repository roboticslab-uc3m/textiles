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
    G = data[:,3]
    N = G/G.max()  # normalize 0..1
    plt.figure()
    plt.hist(G, bins=20)
    plt.figure()
    ax.xaxis.set_ticks(np.arange(-1, 1, 0.1))
    ax.yaxis.set_ticks(np.arange(-1, 1, 0.1))
    ax.set_autoscalex_on(False)
    ax.set_autoscaley_on(False)
    ax.scatter(X,Y,Z, c=N, cmap=plt.cm.spectral)

    # surf = ax.plot_surface(
    #     X, Y, Z, rstride=1, cstride=1,
    #     facecolors=plt.cm.jet(N),
    #     linewidth=0, antialiased=False, shade=False)
    plt.show()


def colorize_point_cloud(data, output_file):
    X = data[:,0]
    Y = data[:,1]
    Z = data[:,2]
    r_min = data[:,3]
    # r_max = data[:,4]

    r_min_norm = (r_min - r_min.min()) / (r_min.max()-r_min.min())
    # r_max_norm = (r_max - r_max.min()) / (r_max.max()-r_max.min())

    r_min_rgba = pylab.cm.jet(r_min_norm)
    # r_max_rgba = pylab.cm.spectral(r_max_norm)


    with open(output_file, 'w') as f:
        # Write pcd header
        f.write(
        """# PCD v.7 - Point Cloud Data file format
        VERSION .7
        FIELDS x y z rgb
        SIZE 4 4 4 4
        TYPE F F F F
        COUNT 1 1 1 1
        WIDTH {0:d}
        HEIGHT 1
        VIEWPOINT 0 0 0 1 0 0 0
        POINTS {0:d}
        DATA ascii""".format(len(r_min_rgba)-1))

        # Write data
        for x, y, z, color in zip(X, Y, Z, r_min_rgba):
            color_packed = int(color[0]) << 16 | int(color[1]) << 8 | int(color[2])
            f.write("{:f} {:f} {:f} {:f}\n".format(x, y, z, color_packed))


if __name__ == '__main__':
    src = np.loadtxt('../pcl/build/cube.m')
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

    data = np.loadtxt('../pcl/build/cube.m')
    # surface_plot(data)

    colorize_point_cloud(data, 'cloud.pcd')
