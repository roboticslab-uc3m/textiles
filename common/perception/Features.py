import numpy as np


def save_SIFT(filename, keypoints, descriptors, class_id_filename="" ):
    """
    Save the computed SIFT descriptors to a numpy file (.npz)
    :param filename: Name of the output file
    :param keypoints: Keypoints (computed with OpenCV)
    :param descriptors: Descriptors (computed with OpenCV)
    :param: class_id_filename: If provided, saves also the class_id of each keypoint in a separate file
    """
    n = len(keypoints)
    m = descriptors[0].shape[0]
    print(m)
    out_array = np.zeros((n, 4+m), dtype=np.float)
    for i, (kp, des) in enumerate(zip(keypoints, descriptors)):
        out_array[i, :4] = kp.pt[0], kp.pt[1], kp.size, kp.angle,
        out_array[i, 4:] = des

    np.savez(filename, descriptors=out_array)

    if class_id_filename:
        class_ids = [kp.class_id for kp in keypoints]
        np.savez(class_id_filename, y=np.array(class_ids))


