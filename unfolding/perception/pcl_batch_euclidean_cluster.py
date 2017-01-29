"""
pcl_batch_euclidean_cluster allows processing several files using the clustering
program clusteringCleanup, developed with pcl
"""

__author__ = 'def'

import os
import glob
import subprocess
import numpy as np
from pcl_utils import  colorize_point_cloud

parallelize = True # True to run several processes in parallel

# garment_data_names = ["Jacket1", "Jacket2", "Jacket3", "Jacket4", "Jacket5",
#                       "LeatherJacket1", "LeatherJacket2", "LeatherJacket3", "LeatherJacket4",
#                       "Hoodie1", "Hoodie2", "Hoodie3",
#                       "Parka1", "Parka2",
#                       "TestBox"]
garment_data_names = ["dress1", "dress2", "dress3", "hoodie1", "hoodie2", "hoodie3", "hoodieBad",
                      "pants1", "pants2", "pants3"]
#garment_data_pattern = "~/Research/point_clouds/{0}/mesh_1.ply"
garment_data_pattern = "~/Research/jresearch/2016-05-06-textiles-draft/{0}/mesh_1.ply"

pcl_processing_binary = "clusteringCleanup"
pcl_processing_folder = "~/Repositories/textiles/pcl/build"

pcl_renderer_binary = "render2png"
pcl_renderer_folder = "~/Repositories/textiles/pcl/build"

output_folder = "~/Repositories/textiles/data/clustered-2016-05-06-thick-garments"

def process_single_cloud(args):
    name, debug = args

    if debug:
        print("Garment: " + name)

    # Create output dir if needed (and empty old data)
    current_output_folder = os.path.expanduser(os.path.join(output_folder, name))
    if not os.path.exists(current_output_folder):
        os.makedirs(current_output_folder)
    else:
        for filename in glob.glob(os.path.join(current_output_folder, "*")):
            try:
                os.remove(filename)
            except OSError as e:
                pass

    # Call the processing program:
    args = [os.path.expanduser(os.path.join(pcl_processing_folder, pcl_processing_binary)),
            "-o",
            current_output_folder,
            os.path.expanduser(garment_data_pattern.format(name))]
    p = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out, err = p.communicate()
    if debug:
        print("Command: " + str(" ".join(args)))
        print("Processing...\n" + str(out))
        print(str(err))
    # Take all point clouds and generate a render
    args = [os.path.expanduser(os.path.join(pcl_renderer_folder, pcl_renderer_binary)),
            "-o", os.path.join(current_output_folder, "render.png")]
    args += glob.glob(os.path.join(current_output_folder, "*.pcd"))
    p = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out, err = p.communicate()
    if debug:
        print("Generate picture from input data...\n" + str(out))
        print(str(err))


if __name__ == "__main__":
    if parallelize:
        from multiprocessing import Pool
        from itertools import repeat

        from tqdm import tqdm

        p = Pool()
        it = p.imap_unordered(process_single_cloud, zip(garment_data_names, repeat(False)))

        for result in tqdm(it):
            pass
    else:
        for name in garment_data_names:
            process_single_cloud(name)