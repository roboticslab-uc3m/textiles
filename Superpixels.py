import numpy as np

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
    labels = np.unique(slic_image)
    avg = np.zeros(slic_image.shape, np.uint8)

    for label in labels:
        avg_value = np.median(np.extract(slic_image == label,image))
        avg = np.where(slic_image == label, int(avg_value), avg)

    return avg

