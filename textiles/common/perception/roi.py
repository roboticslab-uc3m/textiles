# coding=utf-8

"""
Utils to work with Regions of Interest (ROIs)
"""


def load_roi_from_file(filename):
    """
    Loads region of interest from a file. The file format is the following:
    start_x start_y
    end_x end_y
    :param filename: Path to the file
    :return: a region of interest described as two points -> (start_x, start_y), (end_x, end_y)
    """

    with open(filename, 'r') as f:
        lines = f.readlines()
        (start_x, start_y), (end_x, end_y) = [map(int, line.split(' ')) for line in lines]

    return (start_x, start_y), (end_x, end_y)


def crop_roi(roi_rect, image):
    """
    Uses a ROI rectangle to crop a Region Of Interest of the image
    """
    return image[roi_rect[0][1]:roi_rect[1][1], roi_rect[0][0]:roi_rect[1][0]]
