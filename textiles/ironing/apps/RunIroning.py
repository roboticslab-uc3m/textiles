import os
import subprocess
import logging

import begin

from textiles.ironing.perception.ClusteringPointCloudSegmentation import clustering_point_cloud_segmentation_from_file
from textiles.ironing.perception.WrinkleDetection import detect_wrinkles_from_file

pcl_segmentation_binary = "garmentSegmentation"
pcl_cleanup_binary = "garmentCleanup"
pcl_detection_binary = "wrinkleDetection"
pcl_processing_folder = "~/Repositories/textiles/build/textiles/ironing/perception/"

segmented_file_suffix = "-unsegmented.pcd"
cluster_file_suffix = "-cluster0.pcd"
cleaned_file_suffix = "-output.pcd"


@begin.start(auto_convert=True)
@begin.logging
def main(input_file, debug=False):
    input_file_absolute = os.path.abspath(os.path.expanduser(input_file))
    input_folder, input_filename = os.path.split(input_file_absolute)

    # Call the processing program for segmentation
    args = [os.path.expanduser(os.path.join(pcl_processing_folder, pcl_segmentation_binary)),
            "--hsv-s-threshold",  # This is not really used, since we are using clustering
            str(0.4),
            "--hsv-v-threshold",  # This is not really used, since we are using clustering
            str(0.30),
            input_file_absolute]

    p = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out, err = p.communicate()

    if debug:
        print(str(out))
        print(str(err))

    # Do clustering with Python
    clustering_point_cloud_segmentation_from_file(input_file_absolute+segmented_file_suffix)

    # Call the processing program for cleanup
    args = [os.path.expanduser(os.path.join(pcl_processing_folder, pcl_cleanup_binary)),
            input_file_absolute+segmented_file_suffix+cluster_file_suffix]

    p = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out, err = p.communicate()

    if debug:
        print(str(out))
        print(str(err))

    # Call the processing program for computing WiLD descriptors
    args = [os.path.expanduser(os.path.join(pcl_processing_folder, pcl_detection_binary)),
            "--normal-threshold",
            str(0.03),
            input_file_absolute+segmented_file_suffix+cluster_file_suffix+cleaned_file_suffix]

    p = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out, err = p.communicate()

    if debug:
        print(str(out))
        print(str(err))

    # Extract ironing path with Python
    trajectory = detect_wrinkles_from_file(input_file_absolute + segmented_file_suffix + cluster_file_suffix +
                                           cleaned_file_suffix, debug=True)
    print(trajectory)