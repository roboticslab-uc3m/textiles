import numpy as np

def save_SIFT(filename, keypoints, descriptors):
    """
    Save the computed SIFT descriptors to a numpy file (.npz)
    :param filename: Name of the output file
    :param keypoints: Keypoints (computed with OpenCV)
    :param descriptors: Descriptors (computed with OpenCV)
    """
    n = len(keypoints)
    m = descriptors[0].shape[0]
    out_array = np.zeros((n, 4+m), dtype=np.float)
    for i, (kp, des) in enumerate(zip(keypoints, descriptors)):
        out_array[i, :4] = kp.pt[0], kp.pt[1], kp.size, kp.angle,
        out_array[i, 4:] =  des

    np.savez(filename, out_array)
