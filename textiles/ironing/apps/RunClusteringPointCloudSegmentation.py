import os
import logging

import begin

from textiles.ironing.perception.ClusteringPointCloudSegmentation import clustering_point_cloud_segmentation_from_file


@begin.start(auto_convert=True)
@begin.logging
def main(input):
    """
    Performs a clustering_point_cloud_segmentation over a file
    :param input: Input point cloud 
    """
    src_file = os.path.abspath(os.path.expanduser(input))
    clustering_point_cloud_segmentation_from_file(src_file)



