__author__ = 'def'

import os
import subprocess
import numpy as np
from pcl_utils import  colorize_point_cloud

garment_data_names = ["Jacket2", "Jacket3", "Jacket4", "Jacket5"]
garment_data_patterns = ["~/Research/point_clouds/{0}/textured_mesh.ply",
                         "~/Research/point_clouds/{0}/mesh_1_cut.ply"]

pcl_processing_binary = "foldingClothesMesh"
pcl_processing_folder = "~/Repositories/textiles/pcl/build"

output_folder = "~/Repositories/textiles/data/view_colored"
output_curvature_data_prefix = "processed_"
output_colored_data_prefix = "color_"
output_histogram_prefix = "histogram_"

pcl_renderer_binary = "render2png"
pcl_renderer_folder = "~/Repositories/textiles/pcl/build"

if __name__ == "__main__":
    for name in garment_data_names:
        # Check which file pattern is applicable to current garment
        try:
            current_input_file = [ os.path.expanduser(pattern.format(name)) for pattern in garment_data_patterns
                                   if os.path.exists(os.path.expanduser(pattern.format(name)))][0]
        except IndexError:
            print "No file found for " + name + ', skipping...'
            continue

        # Call the processing program:
        args = [ os.path.expanduser(os.path.join(pcl_processing_folder, pcl_processing_binary)),
                 "-t",  str(0.01),
                 current_input_file,
                 "--histogram",
                 os.path.expanduser(os.path.join(output_folder, output_histogram_prefix + name + ".m" )),
                 "-r",
                 os.path.expanduser(os.path.join(output_folder, output_curvature_data_prefix + name + ".m")),
                 "--rsd-params",
                "0.03 0.02 0.2"]
        print "Command: " + str(" ".join(args))
        print "Processing..."
        p = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        out, err = p.communicate()

        print str(out)
        print str(err)

        # Call the coloring routine
        data = np.loadtxt(os.path.expanduser(os.path.join(output_folder,
                                                          output_curvature_data_prefix + name + ".m")))
        colorize_point_cloud(data[:,0:3], data[:,3], os.path.expanduser(
            os.path.join(output_folder, output_colored_data_prefix + "r_min_" + name + ".pcd")))
        colorize_point_cloud(data[:,0:3], data[:,4], os.path.expanduser(
            os.path.join(output_folder, output_colored_data_prefix + "r_max_" + name + ".pcd")))

        # Take all point clouds and generate a render
        args = [os.path.expanduser(os.path.join(pcl_renderer_folder, pcl_renderer_binary)),
                "-o",
                os.path.expanduser(os.path.join(output_folder, "render_" + name + ".png")),
                os.path.expanduser(os.path.join(output_folder,
                                                output_colored_data_prefix + "r_min_" + name + ".pcd"))]
        p = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        out, err = p.communicate()

        print "Generate picture from input data...\n" + str(out)
        print str(err)
