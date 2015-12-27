__author__ = 'def'

import os
import glob
import subprocess
import numpy as np
from pcl_utils import  colorize_point_cloud

garment_data_names = ["Jacket1", "Jacket2", "Jacket3", "Jacket4", "Jacket5"]
garment_data_pattern = "~/Research/point_clouds/{0}/mesh_1_cut.ply"

pcl_processing_binary = "clusteringCleanup"
pcl_processing_folder = "~/Repositories/textiles/pcl/build"

pcl_renderer_binary = "render2png"
pcl_renderer_folder = "~/Repositories/textiles/pcl/build"

output_folder = "~/Repositories/textiles/data/3D data"


if __name__ == "__main__":
    for name in garment_data_names:
        print "Garment: " + name

        # Create output dir if needed (and empty old data)
        current_output_folder = os.path.expanduser(os.path.join(output_folder, name))
        if not os.path.exists(current_output_folder):
            os.makedirs(current_output_folder)
        else:
            for filename in glob.glob(os.path.join(current_output_folder, "*")):
                try:
                    os.remove(filename)
                except OSError, e:
                    pass

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

        # Take all point clouds and generate a render
        args = [os.path.expanduser(os.path.join(pcl_renderer_folder, pcl_renderer_binary)),
                "-o", os.path.join(current_output_folder, "render.png")]
        args += glob.glob(os.path.join(current_output_folder, "*.pcd"))
        p = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        out, err = p.communicate()

        print "Generate picture from input data...\n" + str(out)
        print str(err)