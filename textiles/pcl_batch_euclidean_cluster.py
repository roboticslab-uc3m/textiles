__author__ = 'def'

import os
import subprocess
import numpy as np
from pcl_utils import  colorize_point_cloud

garment_data_names = ["Jacket1", "Jacket2", "Jacket3", "Jacket4", "Jacket5"]
garment_data_pattern = "~/Research/point_clouds/{0}/mesh_1.ply"

pcl_processing_binary = "clusteringCleanup"
pcl_processing_folder = "~/Repositories/textiles/pcl/build"

output_folder = "~/Repositories/textiles/data/3D data"


if __name__ == "__main__":
    for name in garment_data_names:

        # Create output dir if needed
        current_output_folder = os.path.expanduser(os.path.join(output_folder, name))
        if not os.path.exists(current_output_folder):
            os.makedirs(current_output_folder)

        # Call the processing program:
        args = [ os.path.expanduser(os.path.join(pcl_processing_folder, pcl_processing_binary)),
                 "-o",
                 current_output_folder,
                 os.path.expanduser(garment_data_pattern.format(name))]
        p = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        out, err = p.communicate()

        print "Command: " + str(" ".join(args))
        print "Processing...\n" + str(out)
        print str(err)


