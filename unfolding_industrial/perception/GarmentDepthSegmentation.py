import os
import subprocess
from skimage.io import imread

from unfolding.perception.GarmentSegmentation import GarmentSegmentation
from common.perception.Utils import sparse2dense

pcl_processing_binary = "foldingClothesMesh"
pcl_processing_folder = "~/Repositories/textiles/pcl/build"


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
                "--ransac-threshold",
                str(0.0035),
                image]
        p = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        out, err = p.communicate()

        # Load generated mask
        mask = sparse2dense(imread(image+'-mask.png', as_grey=True))

        return mask
