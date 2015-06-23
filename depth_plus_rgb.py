import cv2
import numpy as np
from matplotlib import pyplot as plt
from segmentation import get_coloured_item

__author__ = 'def'

def main():
    image_paths = ['./data/robe01_1_fold.ppm', './data/robe01_2_fold.ppm', './data/sweater02_1_fold.ppm', './data/sweater02_2_fold.ppm', './data/tshirt01_1_fold.ppm',
                   './data/tshirt01_2_fold.ppm','./data/polo01_1_fold.ppm', './data/polo01_2_fold.ppm', './data/dishcloth01_2_fold.ppm', './data/dishcloth01_1_fold.ppm']

    depth_maps = ['./data/robe01_1_fold.mat', './data/robe01_2_fold.mat', './data/sweater02_1_fold.mat', './data/sweater02_2_fold.mat', './data/tshirt01_1_fold.mat',
                   './data/tshirt01_2_fold.mat','./data/polo01_1_fold.mat', './data/polo01_2_fold.mat', './data/dishcloth01_2_fold.mat', './data/dishcloth01_1_fold.mat']

    for path_rgb, path_depth in zip(image_paths, depth_maps):
        # Load image
        image = cv2.imread(path_rgb)

        # Get mask
        mask = get_coloured_item(image)
        cv2.imshow("mask", mask)
        cv2.waitKey(500)
        cv2.destroyAllWindows()

        # Load depth map
        depth_image = np.loadtxt(path_depth)
        print image.shape, image.dtype
        print depth_image.shape, depth_image.dtype
        plt.imshow(np.where(mask[:,:,0]==255,depth_image.transpose(), 0 ))
        plt.show()


if __name__ == '__main__':
    main()