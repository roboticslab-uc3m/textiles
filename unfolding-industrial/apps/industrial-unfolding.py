__author__ = "def"

import cv2
import numpy as np

from unfolding.perception.GarmentSegmentation import GarmentSegmentation
from unfolding.perception.GarmentDepthMapClustering import GarmentDepthMapClustering
from unfolding.perception.GarmentPickAndPlacePoints import GarmentPickAndPlacePoints
from unfolding.perception import GarmentPlot

path_input_mesh = "/home/def/Research/jresearch/2016-05-06-textiles-draft/pants1/textured_mesh.ply"
#path_input_mesh = "/home/def/Research/jresearch/2016-04-20-textiles-draft/hoodie3/textured_mesh.ply"

def sparse2dense(mask):
    """
    From a sparse segmentation mask (obtained from a point cloud) get a dense mask
    """
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Fill large holes
    mask_outlines, dummy = cv2.findContours(closing.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.fillPoly(closing, mask_outlines, 255)

    return closing


if __name__ == "__main__":
    path_rgb_image = path_input_mesh + "-RGB.png" # Theoretically irrelevant to result, but here for retrocompatibility
    path_mask = path_input_mesh + "-mask.png"
    path_depth_image = path_input_mesh + "-depth.txt"

    # Load input data
    image_src = cv2.imread(path_rgb_image)
    depth_image = np.loadtxt(path_depth_image)
    depth_image = depth_image.transpose() # Retrocompatibility again

    # Garment Segmentation Stage
    # Note: in this application the mask is obtained directly from the point
    # cloud depth data
    mask = sparse2dense(cv2.imread(path_mask, cv2.cv.CV_LOAD_IMAGE_GRAYSCALE))
    approximated_polygon = GarmentSegmentation.compute_approximated_polygon(mask)
    GarmentPlot.plot_segmentation_stage(image_src, mask, approximated_polygon)

    # Garment Depth Map Clustering Stage
    preprocessed_depth_image = GarmentDepthMapClustering.preprocess_heightmap(depth_image, mask)
    labeled_image = GarmentDepthMapClustering.cluster_similar_regions(preprocessed_depth_image, mask)
    GarmentPlot.plot_clustering_stage(image_src, labeled_image)

    # Garment Pick and Place Points Stage
    unfold_paths = GarmentPickAndPlacePoints.calculate_unfold_paths(labeled_image, approximated_polygon)
    bumpiness = GarmentPickAndPlacePoints.calculate_bumpiness(labeled_image, unfold_paths)
    pick_point, place_point = GarmentPickAndPlacePoints.calculate_pick_and_place_points(labeled_image, unfold_paths,
                                                                                         bumpiness)
    # GarmentPlot.plot_paths(image_src, approximated_polygon, unfold_paths)
    print "Bumpiness: ", bumpiness
    # GarmentPlot.plot_pick_and_place_points(image_src, pick_point, place_point)
    GarmentPlot.plot_pick_and_place_stage(image_src, labeled_image, approximated_polygon, unfold_paths,
                                                               pick_point, place_point)
