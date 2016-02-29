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
    for path in unfold_paths:
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

def plot_segmentation_stage(image_rgb, mask, polygon, to_file=None):
    plt.figure()
    plt.imshow(image_rgb)
    plt.imshow(mask, cmap=plt.cm.gray, alpha=0.7)
    points = [tuple(point[0]) for point in polygon]
    # Plot lines
    for (start_x, start_y), (end_x, end_y) in zip(points, points[1:]+points[0:1]):
        plt.plot( (start_x, end_x), (start_y, end_y), 'r-', linewidth=2.0, alpha=0.7 )
    # Plot points
    for x, y in points:
        plt.plot(x, y, 'ro', alpha=0.7)
    plt.axis('off')

    if to_file:
        plt.savefig(to_file, bbox_inches='tight')
        plt.close()
    else:
        plt.show()

def plot_clustering_stage(image_rgb, labeled_image, to_file=None):
    plt.figure()
    cax = plt.imshow(labeled_image, cmap=plt.cm.RdGy)
    plt.imshow(image_rgb)
    plt.imshow(labeled_image, cmap=plt.cm.RdGy, alpha=0.6)
    cbar = plt.colorbar(cax, ticks=[labeled_image.min(),labeled_image.max()],
                 orientation ='horizontal', shrink=0.6, pad=0.05)
    cbar.ax.set_xticklabels(['Foreground', 'Background'])
    for tick in cbar.ax.xaxis.get_major_ticks():
                tick.label.set_fontsize(20)

    plt.axis('off')
    plt.tight_layout()

    if to_file:
        plt.savefig(to_file, bbox_inches='tight')
        plt.close()
    else:
        plt.show()

def plot_pick_and_place_stage(image_rgb, labeled_image, approximated_polygon, unfold_paths,
                              pick_point, place_point, to_file=None):
    plot_rgb(image_rgb, show=False)
    plt.imshow(labeled_image, cmap=plt.cm.RdGy, alpha=0.6)
    points = [tuple(point[0]) for point in approximated_polygon]
    # Plot polygon lines
    for (start_x, start_y), (end_x, end_y) in zip(points, points[1:]+points[0:1]):
        plt.plot( (start_x, end_x), (start_y, end_y), 'r-', linewidth=2.0 )
    # Plot polygon points
    for x, y in points:
        plt.plot(x, y, 'ro')
    #Plot unfold paths
    for path in unfold_paths:
        (start_x, start_y), (end_x, end_y) = path
        plt.plot( (start_x, end_x), (start_y, end_y), 'go-', linewidth=2.0 )
    # Plot arrow
    plt.arrow(pick_point[0], pick_point[1], place_point[0]-pick_point[0], place_point[1]-pick_point[1],
               head_width=15, head_length=15, fc='blue', ec='blue', lw=5, alpha=0.7)
    plt.axis('off')

    if to_file:
        plt.savefig(to_file, bbox_inches='tight')
        plt.close()
    else:
        plt.show()