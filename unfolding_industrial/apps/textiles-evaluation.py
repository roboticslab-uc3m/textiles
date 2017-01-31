import numpy as np
import os
from multiprocessing import Pool
from tqdm import tqdm
import matplotlib.pyplot as plt

from unfolding_industrial.perception.GarmentDepthSegmentation import GarmentDepthSegmentation
from unfolding.perception.GarmentDepthMapClustering import GarmentDepthMapClustering
from unfolding_industrial.perception.GarmentMirrorPickAndPlacePoints import GarmentMirrorPickAndPlacePoints
import unfolding.perception.GarmentPlot
from common.perception.Utils import depthMap_2_heightMap
from common.math import normalize

__author__ = "def"

input_folder = '~/Research/datasets/2017-01-30-unfolding'
parallel = True


def compute_stages(point_cloud_path):
    # Output file prefix
    filename = os.path.basename(os.path.dirname(point_cloud_path))
    prefix = os.path.splitext(filename)[0]
    out_prefix = os.path.join(output_dir, prefix)

    # Garment Segmentation Stage
    mask = GarmentDepthSegmentation.background_subtraction(point_cloud_path)

    # Load other computed data
    depth_image = depthMap_2_heightMap(np.loadtxt(point_cloud_path+'-depth.txt'))
    # image_src = plt.get_cmap('RdGy')(depth_image)
    image_src = plt.get_cmap('viridis')(normalize(np.ma.masked_array(depth_image, mask=np.bitwise_not(mask))))
    depth_image = depth_image.transpose()  # Retrocompatibility

    approximated_polygon = GarmentDepthSegmentation.compute_approximated_polygon(mask)
    unfolding.perception.GarmentPlot.plot_segmentation_stage(image_src, mask, approximated_polygon,
                                                             to_file=out_prefix + '-segmentation.png')

    # Garment Depth Map Clustering Stage
    preprocessed_depth_image = GarmentDepthMapClustering.preprocess(depth_image, mask)
    # preprocessed_depth_image = normalize(np.ma.masked_array(depth_image, mask=np.bitwise_not(mask)))
    labeled_image = GarmentDepthMapClustering.cluster_similar_regions(preprocessed_depth_image, mask=mask)
    unfolding.perception.GarmentPlot.plot_clustering_stage(image_src, labeled_image, to_file=out_prefix +
                                                           '-clustering.png')

    try:
        unfold_paths = GarmentMirrorPickAndPlacePoints.calculate_unfold_paths(labeled_image, approximated_polygon)
        bumpiness = GarmentMirrorPickAndPlacePoints.calculate_bumpiness(labeled_image, unfold_paths)
        pick_point, place_point = GarmentMirrorPickAndPlacePoints.calculate_pick_and_place_points(labeled_image,
                                                                                                  unfold_paths,
                                                                                                  bumpiness,
                                                                                                  approximated_polygon=approximated_polygon)
    except ValueError as e:
        print("\t[-] Exception ocurred!", e)
        return False

    f = open(out_prefix + '-bumpiness.txt', 'w')
    for value in bumpiness:
        f.write(str(value)+'\n')
    f.close()
    unfolding.perception.GarmentPlot.plot_pick_and_place_stage(image_src, labeled_image, approximated_polygon,
                                                               unfold_paths,
                                                               pick_point, place_point, to_file=out_prefix + '-pnp.png')
    unfolding.perception.GarmentPlot.plot_pick_and_place_points(image_src, pick_point, place_point, to_file=out_prefix +
                                                                '-direction.png')

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

    if parallel:
        p = Pool()
        it = p.imap_unordered(compute_stages, point_cloud_paths)

        for result in tqdm(it):
            if not result:
                print("Some error occurred!")
    else:
        for path in tqdm(point_cloud_paths):
            result = compute_stages(path)
