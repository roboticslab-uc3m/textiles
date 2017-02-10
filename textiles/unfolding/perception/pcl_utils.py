import struct

import numpy as np
import pylab

import textiles.unfolding.perception.GarmentAnalysis as GarmentAnalysis

__author__ = 'def'


def surface_plot(data):
    # from mpl_toolkits.mplot3d import Axes3D
    import matplotlib.pyplot as plt

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    X = data[:, 0]
    Y = data[:, 1]
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


def rgb_to_pcl_float(r, g, b):
    """
    src: python-pcl bindings
    """
    rgb = int(r*255) << 16 | int(g*255) << 8 | int(b*255)
    rgb_bytes = struct.pack('I', rgb)
    p_rgb = struct.unpack('f', rgb_bytes)[0]
    return p_rgb


def colorize_point_cloud(xyz_data, color_data, output_file, cmap=pylab.cm.cool):
    X = xyz_data[:, 0]
    Y = xyz_data[:, 1]
    Z = xyz_data[:, 2]

    try:
        color_data_norm = (color_data - color_data.min()) / (color_data.max()-color_data.min())
        color_data_rgba = cmap(color_data_norm)
    except RuntimeError as e:
        print("Error colorizing point cloud: \n" + str(e))
        return

    with open(output_file, 'w') as f:
        # Write pcd header
        f.write("""# PCD v.7 - Point Cloud Data file format
VERSION .7
FIELDS x y z rgb
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH {0:d}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {0:d}
DATA ascii
""".format(len(color_data_rgba)-1))

        # Write data
        for x, y, z, color in zip(X, Y, Z, color_data_rgba):
            color_packed = rgb_to_pcl_float(*color[0:3])
            f.write("{:f} {:f} {:f} {:e}\n".format(x, y, z, color_packed))


def colorize_rsd_point_cloud(xyz_data, r_min_data, r_max_data, output_file):
    colors = {"plane": rgb_to_pcl_float(255, 0, 0),
              "cylinder": rgb_to_pcl_float(0, 0, 255),
              "edge": rgb_to_pcl_float(0, 255, 0),
              "sphere": rgb_to_pcl_float(0, 255, 255),
              "noise": rgb_to_pcl_float(255, 255, 255)}

    X = xyz_data[:, 0]
    Y = xyz_data[:, 1]
    Z = xyz_data[:, 2]

    with open(output_file, 'w') as f:
        # Write pcd header
        f.write("""# PCD v.7 - Point Cloud Data file format
VERSION .7
FIELDS x y z rgb
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH {0:d}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {0:d}
DATA ascii
""".format(len(r_min_data)-1))

        # Write data
        for x, y, z, r_min, r_max in zip(X, Y, Z, r_min_data, r_max_data):
            # Decision tree:
            if r_min > 0.1:
                color = colors["plane"]
            elif r_max > 0.15:
                color = colors["cylinder"]
            elif r_max > r_min + 0.05:
                color = colors["edge"]
            else:
                color = colors["sphere"]

            f.write("{:f} {:f} {:f} {:e}\n".format(x, y, z, color))

if __name__ == '__main__':
    RSD = True
    WILD = True

    if RSD:
        # data = np.loadtxt('../pcl/build/curvature_data.m')
        data = np.loadtxt('../../../build/ironing/perception/rsd_wrinkle.m')

        colorize_point_cloud(data[:,0:3], data[:,3], 'cloud-r_min.pcd', cmap=pylab.cm.RdGy)
        colorize_point_cloud(data[:,0:3], data[:,4], 'cloud-r_max.pcd', cmap=pylab.cm.RdGy)
        colorize_rsd_point_cloud(data[:,0:3], data[:,3], data[:,4], 'color-rsd.pcd')

    if WILD:
        data = np.loadtxt('../../../build/ironing/perception/wild_descriptors.m')
        colorize_point_cloud(data[:,0:3], data[:,3], 'cloud-wild.pcd', cmap=pylab.cm.RdGy)