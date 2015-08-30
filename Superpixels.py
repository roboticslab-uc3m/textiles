import numpy as np
from skimage.measure import moments

__author__ = 'def'


def get_masks_slic(slic_image):
    labels = np.unique(slic_image)
    masks = []

    for label in labels:
        # Copy image and do threshold
        mask = np.where(slic_image == label, 255, 0)
        masks.append(mask)

    return masks

def get_average_slic(image, slic_image):
    """ Returns an image with the median value for each superpixel """
    labels = np.unique(slic_image)
    avg = np.zeros(slic_image.shape, np.uint8)

    for label in labels:
        avg_value = np.median(np.extract(slic_image == label,image))
        avg = np.where(slic_image == label, int(avg_value), avg)

    return avg

def get_highest_superpixel(image):
    """ Returns the superpixel with the lowest value (highest in the point cloud) """
    labels = np.unique(image)
    lowest_value = labels[np.unravel_index(labels.argmin(), labels.shape)]
    return np.where(image<=lowest_value, 255, 0).astype(np.uint8)

def get_centroid(image):
    m = moments(image)
    return (m[0, 1] / m[0, 0], m[1, 0] / m[0, 0])


def line_sampling_points(p1, p2, step):
    # Order points:
    if p1[0] <= p2[0]:
        start = p1
        end = p2
    else:
        start = p2
        end = p1

    # Find line slope and length
    slope = np.true_divide(end[1] - start[1], end[0] - start[0])
    length = np.sqrt(np.power(end[1] - start[1], 2) + np.power(end[0] - start[0], 2))
    num_samples = int(length / step)
    print 'Slope: %f Length: %f Num samples: %f' % (slope, length, num_samples)
    # Obtain points to sample
    x = np.linspace(start[0], end[0], num_samples)
    y = (x-start[0]) * slope + start[1]
    return x, y


def line_sampling(image, p1, p2, step):
    x, y = line_sampling_points(p1, p2, step)
    # print image.shape
    if len(image.shape) == 3:
        return [ image[int(j), int(i), :] for i, j in zip(x, y)]
    else:
        return [ image[int(j), int(i)] for i, j in zip(x, y)]



