import os
import subprocess
import cv2

from unfolding.perception.GarmentSegmentation import GarmentSegmentation
from common.perception.Utils import sparse2dense

pcl_processing_binary = "kinfuUnfolding"
pcl_processing_folder = "~/Repositories/textiles/build/unfolding-industrial/perception/"


class GarmentDepthSegmentation(GarmentSegmentation):
    @staticmethod
    def background_subtraction(image: 'Point Cloud filepath'):
        """
        Segments the garment from the background. This implementation takes a point cloud
        and uses RANSAC to segment it from the table
        :param image: Point Cloud filepath
        :return: Segmentation mask where white is garment and black is background
        """
        # Process input 'image'
        args = [os.path.expanduser(os.path.join(pcl_processing_folder, pcl_processing_binary)),
                image,
                "--ransac-threshold",
                str(0.0035)
                ]
        p = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        out, err = p.communicate()

        # Load generated mask
        mask = sparse2dense(cv2.imread(image+'-mask.png', cv2.IMREAD_GRAYSCALE))

        return mask
