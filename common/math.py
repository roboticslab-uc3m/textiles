import numpy as np


def rotX(angle):
    radians = np.deg2rad(angle)
    s = np.sin(radians)
    c = np.cos(radians)

    return np.array([[1, 0,  0],
                     [0, c, -s],
                     [0, s,  c]])


def rotY(angle):
    radians = np.deg2rad(angle)
    s = np.sin(radians)
    c = np.cos(radians)

    return np.array([[ c, 0, s],
                     [ 0, 1, 0],
                     [-s, 0, c]])


def rotZ(angle):
    radians = np.deg2rad(angle)
    s = np.sin(radians)
    c = np.cos(radians)

    return np.array([[c, -s, 0],
                     [s,  c, 0],
                     [0,  0, 1]])


def normalize(X):
    """
    Normalize an array to have values between 0 and 1
    :param array: Input array
    :return: Normalized array
    """
    return (X - X.min()) / (X.max() - X.min())


def mirror(v, l):
    """
    Computes the reflection of point v across a line through the origin in two dimensions
    :param v: Point to compute reflection
    :param l: Line through the origin
    """
    return 2*np.dot(v, l)*l/np.dot(l, l) - v