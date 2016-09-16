import os
import cv2
import numpy as np
import matplotlib.pyplot as plt

from common.perception.Transformer import Transformer
from common.perception.depth_calibration import H_root_cam, kinfu_wrt_cam
from common.perception.Utils import points_to_file

path_input_mesh = "/home/def/Research/jresearch/2016-09-06-textiles-unfolding/calibration5/mesh_1.ply"

if __name__ == "__main__":
    # Convert pick and place points to robot frame
    change_frame = Transformer()

    # Load configuration data and configure
    with open(path_input_mesh + "-origin.txt", "r") as f:
        file_contents = f.readlines()
        image_origin = [ float(i) for i in file_contents[0].split(' ')]
    change_frame.add_image_params((image_origin[0], image_origin[1]), 0.005)

    kinfu_wrt_object = np.loadtxt(path_input_mesh+"-transform.txt")
    change_frame.add_kinfu_wrt_object_transform(kinfu_wrt_object)
    change_frame.add_kinfu_params(np.linalg.inv(kinfu_wrt_cam))
    change_frame.add_cam_wrt_root_transform(H_root_cam)

    print "---"

    test_points_px = [(139, 60), (68,49), (40,113), (122,131), (100, 78)]
    points_to_file(change_frame.debug(test_points_px), os.path.join(os.path.split(path_input_mesh)[0], "points.pcd"))

    test_points = change_frame.root(test_points_px)
    for point in test_points:
        print point

    print "---"

    # Convert mask to root
    path_mask = path_input_mesh + "-mask.png"
    mask = cv2.imread(path_mask, cv2.cv.CV_LOAD_IMAGE_GRAYSCALE)
    mask_px = [(x, y) for x in range(mask.shape[1]) for y in range(mask.shape[0]) if mask[y, x] == 255]
    mask_points = change_frame.root(mask_px)

    # Convert grid
    px = [(x, y) for x in np.arange(0, mask.shape[1], 5) for y in np.arange(0, mask.shape[0], 5)]
    points = change_frame.root(px)

    # Plot good points
    from common.perception.depth_calibration import points_root
    plt.scatter([p[0] for p in points], [p[1] for p in points], c='b')
    plt.scatter([p[0] for p in mask_points], [p[1] for p in mask_points], c='r')
    plt.scatter([p[0] for p in test_points], [p[1] for p in test_points], c='y')
    plt.plot([p[0] for p in points_root], [p[1] for p in points_root], 'go-')
    plt.show()
