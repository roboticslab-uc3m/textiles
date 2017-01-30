from skimage.io import imread
import numpy as np
import os
from multiprocessing import Pool
from tqdm import tqdm

from unfolding_industrial.perception.GarmentDepthSegmentation import GarmentDepthSegmentation
from unfolding.perception.GarmentDepthMapClustering import GarmentDepthMapClustering
from unfolding.perception.GarmentPickAndPlacePoints import GarmentPickAndPlacePoints
import unfolding.perception.GarmentPlot
from unfolding.perception.GarmentUtils import load_data

__author__ = "def"

input_folder = '~/Research/datasets/2017-01-30-unfolding'

def compute_stages(point_cloud_path):
    # Output file prefix
    filename = os.path.basename(point_cloud_path)
    prefix = os.path.splitext(filename)[0]
    out_prefix = os.path.join(output_dir, prefix)

    # Garment Segmentation Stage
    mask = GarmentDepthSegmentation.background_subtraction(point_cloud_path)

    # Load other computed data
    depth_image = np.loadtxt(point_cloud_path+'-depth.txt')
    depth_image = depth_image.transpose()  # Retrocompatibility
    image_src = mask # To be computed from depth image


    approximated_polygon = GarmentDepthSegmentation.compute_approximated_polygon(mask)
    unfolding.perception.GarmentPlot.plot_segmentation_stage(image_src, mask, approximated_polygon,
                                                             to_file=out_prefix + '-segmentation.pdf')

    # Garment Depth Map Clustering Stage
    preprocessed_depth_image = GarmentDepthMapClustering.preprocess(depth_image, mask)
    labeled_image = GarmentDepthMapClustering.cluster_similar_regions(preprocessed_depth_image)
    unfolding.perception.GarmentPlot.plot_clustering_stage(image_src, labeled_image, to_file=out_prefix +
                                                           '-clustering.pdf')
    # Garment Pick and Place Points Stage
    try:
        unfold_paths = GarmentPickAndPlacePoints.calculate_unfold_paths(labeled_image, approximated_polygon)
        bumpiness = GarmentPickAndPlacePoints.calculate_bumpiness(labeled_image, unfold_paths)
        pick_point, place_point = GarmentPickAndPlacePoints.calculate_pick_and_place_points(labeled_image, unfold_paths,
                                                                                            bumpiness)
    except ValueError as e:
        # print("\t[-] Exception ocurred!", e)
        return False

    f = open(out_prefix + '-bumpiness.txt', 'w')
    for value in bumpiness:
        f.write(str(value)+'\n')
    f.close()
    unfolding.perception.GarmentPlot.plot_pick_and_place_stage(image_src, labeled_image, approximated_polygon,
                                                               unfold_paths,
                                                               pick_point, place_point, to_file=out_prefix + '-pnp.pdf')
    unfolding.perception.GarmentPlot.plot_pick_and_place_points(image_src, pick_point, place_point, to_file=out_prefix +
                                                                '-direction.pdf')

    return True


if __name__ == "__main__":
    # Input and output folders:
    input_dir = os.path.expanduser(input_folder)
    print("[+] Loading data from:", input_dir)
    output_dir = input_dir+'-results'
    if not os.path.exists(output_dir):
            os.makedirs(output_dir)

    # Load input paths
    point_cloud_paths = [os.path.join(input_dir, i, 'mesh_1.ply') for i in os.listdir(input_dir)
                         if os.path.isdir(os.path.join(input_dir, i)) and not i.startswith('.')]

    if not point_cloud_paths:
        print("[!] No data was loaded!")
    else:
        print("[+] Loaded: {} garments".format(len(point_cloud_paths)))

    p = Pool()
    it = p.imap_unordered(compute_stages, point_cloud_paths)

    for result in tqdm(it):
        pass
