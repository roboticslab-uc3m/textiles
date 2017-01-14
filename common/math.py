import numpy as np

def rotX(angle):
    radians = np.deg2rad(angle)
    s = np.sin(radians)
    c = np.cos(radians)

    return np.array([[ 1, 0,  0],
                     [ 0, c, -s],
                     [ 0, s,  c]])

def rotY(angle):
    radians = np.deg2rad(angle)
    s = np.sin(radians)
    c = np.cos(radians)

    return np.array([[  c, 0, s],
                     [  0, 1, 0],
                     [ -s, 0, c]])

def rotZ(angle):
    radians = np.deg2rad(angle)
    s = np.sin(radians)
    c = np.cos(radians)

    return np.array([[ c, -s, 0],
                     [ s,  c, 0],
                     [ 0,  0, 1]])

def normalize_array(array):
    """
    Normalize an array to have values between 0 and 1
    Note: this only works with arrays with values from 0 to any positive values.
    :param array: Input array
    :return: Normalized array
    """
    array -= array.min()
    array /= array.max()
    return array

def normalize(X):
    """
    Normalize an array to have values between 0 and 1
    :param array: Input array
    :return: Normalized array
    """
    return (X - X.min()) / (X.max() - X.min())