__author__ = "def"

import cv2
import numpy as np
import os
from multiprocessing import Pool
from tqdm import tqdm

from GarmentSegmentation import GarmentSegmentation
from GarmentDepthMapClustering import GarmentDepthMapClustering
from GarmentPickAndPlacePoints import GarmentPickAndPlacePoints
import GarmentPlot
from utils import load_data


def compute_stages(args):
    path_rgb_image, path_depth_image = args

    # Output file prefix
    filename = os.path.basename(path_rgb_image)
    prefix = os.path.splitext(filename)[0]
    out_prefix = os.path.join(output_dir, prefix)

    # Load input data
    image_src = cv2.imread(path_rgb_image)
    depth_image = np.loadtxt(path_depth_image)

    # Garment Segmentation Stage
    mask = GarmentSegmentation.background_substraction(image_src)
    approximated_polygon = GarmentSegmentation.compute_approximated_polygon(mask)
    GarmentPlot.plot_segmentation_stage(image_src, mask, approximated_polygon,
                                        to_file=out_prefix + '-segmentation.png')

    # Garment Depth Map Clustering Stage
    preprocessed_depth_image = GarmentDepthMapClustering.preprocess(depth_image, mask)
    labeled_image = GarmentDepthMapClustering.cluster_similar_regions(preprocessed_depth_image)
    GarmentPlot.plot_clustering_stage(image_src, labeled_image, to_file=out_prefix + '-clustering.png')
    # Garment Pick and Place Points Stage
    try:
        unfold_paths = GarmentPickAndPlacePoints.calculate_unfold_paths(labeled_image, approximated_polygon)
        bumpiness = GarmentPickAndPlacePoints.calculate_bumpiness(labeled_image, unfold_paths)
        pick_point, place_point = GarmentPickAndPlacePoints.calculate_pick_and_place_points(labeled_image, unfold_paths,
                                                                                            bumpiness)
    except ValueError, e:
        # print "\t[-] Exception ocurred!", e
        return False

    f = open(out_prefix + '-bumpiness.txt', 'w')
    for value in bumpiness:
        f.write(str(value)+'\n')
    f.close()
    GarmentPlot.plot_pick_and_place_stage(image_src, labeled_image, approximated_polygon, unfold_paths,
                                          pick_point, place_point, to_file=out_prefix + '-pnp.png')

    return True


if __name__ == "__main__":
    # Input and output folders:
    input_dir = os.path.expanduser('~/Research/garments-birdsEye-flat')
    print "[+] Loading data from:", input_dir
    output_dir = input_dir+'-results'
    if not os.path.exists(output_dir):
            os.makedirs(output_dir)

    # Load input paths
    image_paths, depth_image_paths = load_data(input_dir)
    if not image_paths or not depth_image_paths:
        print "[!] No data was loaded!"
    else:
        print "[+] Loaded: {} garments".format(len(image_paths))

    # for path_rgb_image, path_depth_image in zip(image_paths, depth_image_paths):
    #     compute_stages(path_rgb_image, path_depth_image)

    # p = Pool(8)
    # p.map(compute_stages, zip(image_paths, depth_image_paths))
    # p.close()

    p = Pool()
    it = p.imap_unordered(compute_stages, zip(image_paths, depth_image_paths))

    for result in tqdm(it):
        pass