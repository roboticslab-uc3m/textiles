import cv2
import numpy as np

from textiles.unfolding.perception.GarmentSegmentation import GarmentSegmentation
from textiles.unfolding.perception.GarmentDepthMapClustering import GarmentDepthMapClustering
from textiles.unfolding.perception.GarmentPickAndPlacePoints import GarmentPickAndPlacePoints
import textiles.unfolding.perception.GarmentPlot as GarmentPlot
from textiles.unfolding.perception.GarmentUtils import load_data

__author__ = "def"

def main():
    # image_paths, depth_image_paths = load_data('../data/20150902')
    image_paths, depth_image_paths = load_data('/home/def/Research/garments-birdsEye/jacket')

    for path_rgb_image, path_depth_image in zip(image_paths, depth_image_paths):
        # Load input data
        image_src = cv2.imread(path_rgb_image)
        depth_image = np.loadtxt(path_depth_image)

        # Garment Segmentation Stage
        mask = GarmentSegmentation.background_subtraction(image_src)
        approximated_polygon = GarmentSegmentation.compute_approximated_polygon(mask)
        GarmentPlot.plot_segmentation_stage(image_src, mask, approximated_polygon)

        # Garment Depth Map Clustering Stage
        preprocessed_depth_image = GarmentDepthMapClustering.preprocess(depth_image, mask)
        labeled_image = GarmentDepthMapClustering.cluster_similar_regions(preprocessed_depth_image)
        GarmentPlot.plot_clustering_stage(image_src, labeled_image)

        # Garment Pick and Place Points Stage
        unfold_paths = GarmentPickAndPlacePoints.calculate_unfold_paths(labeled_image, approximated_polygon)
        bumpiness = GarmentPickAndPlacePoints.calculate_bumpiness(labeled_image, unfold_paths)
        pick_point, place_point = GarmentPickAndPlacePoints.calculate_pick_and_place_points(labeled_image, unfold_paths,
                                                                                            bumpiness)
        # GarmentPlot.plot_paths(image_src, approximated_polygon, unfold_paths)
        print("Bumpiness: ", bumpiness)
        # GarmentPlot.plot_pick_and_place_points(image_src, pick_point, place_point)
        GarmentPlot.plot_pick_and_place_stage(image_src, labeled_image, approximated_polygon, unfold_paths,
                                              pick_point, place_point)


if __name__ == "__main__":
    main()
