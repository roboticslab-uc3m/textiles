__author__="def"

import matplotlib.pyplot as plt


def plot_rgb(image, show=True, **kwargs):
    plt.figure()
    plt.imshow(image, **kwargs)
    if show:
        plt.show()

def plot_depth(image, show=True):
    plt.figure()
    plt.imshow(image, cmap=plt.cm.RdGy)
    plt.colorbar(ticks=[0, 500, image.max()], orientation ='horizontal', shrink=0.75, pad=0.01)
    plt.axis('off')
    plt.tight_layout()
    if show:
        plt.show()

def plot_mask(image, show=True):
    plt.figure()
    plt.imshow(image, cmap=plt.cm.gray)
    plt.axis('off')
    if show:
        plt.show()

def plot_contour(image, contour, color='r', show=True):
    plot_rgb(image, show=False)
    points = [tuple(point[0]) for point in contour]
    # Plot lines
    for (start_x, start_y), (end_x, end_y) in zip(points, points[1:]+points[0:1]):
        plt.plot( (start_x, end_x), (start_y, end_y), color+'-', linewidth=2.0 )
    # Plot points
    for x, y in points:
        plt.plot(x, y, color+'o')
    plt.axis('off')
    if show:
        plt.show()

def plot_paths(image_src, approximated_polygon, unfold_paths, show=True):
    plot_contour(image_src, approximated_polygon, show=False)
    for i, path in unfold_paths:
        if i:
            (start_x, start_y), (end_x, end_y) = path
            plt.plot( (start_x, end_x), (start_y, end_y), 'go-', linewidth=2.0 )
    plt.axis('off')
    if show:
        plt.show()

def plot_pick_and_place_points(image_src, pick_point, place_point, show=True):
    plt.imshow(image_src, cmap=plt.cm.gray)
    plt.axis('off')
    plt.arrow(pick_point[0], pick_point[1], place_point[0]-pick_point[0], place_point[1]-pick_point[1],
               head_width=15, head_length=15, fc='blue', ec='blue', lw=5)
    if show:
        plt.show()