__author__ = 'def'

import os
import subprocess
import numpy as np
from pcl_utils import  colorize_point_cloud

garment_data_names = ["chupa1", "chupa2", "chupa3", "chupa4", "sudadera1"]
garment_data_pattern = "~/Research/point_clouds/{0}/textured_mesh.ply"

pcl_processing_binary = "foldingClothes"
pcl_processing_folder = "~/Repositories/textiles/pcl/build"

output_folder = "~/Repositories/textiles/data/view_colored"
output_curvature_data_prefix = "processed_"
output_colored_data_prefix = "color_"
output_depthmap_prefix = "depth_"

if __name__ == "__main__":
    for name in garment_data_names:
        # Call the processing program:
        args = [ os.path.expanduser(os.path.join(pcl_processing_folder, pcl_processing_binary)),
                 "-t",  str(0.02),
                 os.path.expanduser(garment_data_pattern.format(name)),
                 "-d",
                 os.path.expanduser(os.path.join(output_folder, output_depthmap_prefix + name + ".m" )),
                 "-r",
                 os.path.expanduser(os.path.join(output_folder, output_curvature_data_prefix + name + ".m"))]
        p = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        out, err = p.communicate()

        print "Command: " + str(" ".join(args))
        print "Processing...\n" + str(out)
        print str(err)

        # Call the coloring routine
        data = np.loadtxt(os.path.expanduser(os.path.join(output_folder,
                                                          output_curvature_data_prefix + name + ".m")))
        colorize_point_cloud(data[:,0:3], data[:,3], os.path.expanduser(
            os.path.join(output_folder, output_colored_data_prefix + "r_min_" + name + ".pcd")))
        colorize_point_cloud(data[:,0:3], data[:,4], os.path.expanduser(
            os.path.join(output_folder, output_colored_data_prefix + "r_max_" + name + ".pcd")))


