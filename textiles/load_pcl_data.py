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

def colorize_point_cloud(xyz_data, color_data, output_file, cmap=pylab.cm.cool):
    X = xyz_data[:,0]
    Y = xyz_data[:,1]
    Z = xyz_data[:,2]

    try:
        color_data_norm = (color_data - color_data.min()) / (color_data.max()-color_data.min())
        color_data_rgba = cmap(color_data_norm)
    except RuntimeError, e:
        print "Error colorizing point cloud: \n" + str(e)
        return

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
        DATA ascii""".format(len(color_data_rgba)-1))

        # Write data
        for x, y, z, color in zip(X, Y, Z, color_data_rgba):
            color_packed = int(color[0]*255) << 16 | int(color[1]*255) << 8 | int(color[2]*255)
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

    data = np.loadtxt('../pcl/build/rsd_data2.m')
    # surface_plot(data)

    colorize_point_cloud(data[:,0:3], data[:,3], 'cloud-r_min.pcd')
    colorize_point_cloud(data[:,0:3], data[:,4], 'cloud-r_max.pcd')