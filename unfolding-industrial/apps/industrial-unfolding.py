__author__ = "def"

import cv2
import numpy as np
import os

# Local (textiles) imports
from unfolding.perception.GarmentSegmentation import GarmentSegmentation
from unfolding.perception.GarmentDepthMapClustering import GarmentDepthMapClustering
from unfolding.perception.GarmentPickAndPlacePoints import GarmentPickAndPlacePoints
from unfolding.perception import GarmentPlot
from common.perception.Transformer import Transformer
from common.perception.depth_calibration import H_root_cam, kinfu_wrt_cam
from common.user_interface import query_yes_no
import abb


#path_input_mesh = "/home/def/Research/jresearch/2016-05-06-textiles-draft/pants1/textured_mesh.ply"
#path_input_mesh = "/home/def/Research/jresearch/2016-04-20-textiles-draft/hoodie3/textured_mesh.ply"
path_input_mesh = "/home/yo/martes/blackHoodie3/mesh_1.ply"

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
    # Connect to robot
    robot = abb.Robot('192.168.125.1')
    robot.set_units('meters', 'degrees')
    ans = query_yes_no("Go to starting point? (WARNING: this operation might be potentially destructive)", default="no")
    if ans == 'yes':
        robot.set_tool([[0,0,1.000],[1,0,0,0]])
        robot.set_cartesian([[1.01784,-0.28281,0.06031],[0.0495452, 0.00703024, 0.998742, 0.00318766]])

    ans = query_yes_no("Perform Scan? (WARNING: this operation might be potentially destructive)", default="no")
    if ans == 'yes':
        robot.scan()

    query_yes_no("Start processing?", default="yes")

    path_mask = path_input_mesh + "-mask.png"
    path_depth_image = path_input_mesh + "-depth.txt"

    # Load input data
    depth_image = np.loadtxt(path_depth_image)
    depth_image = depth_image.transpose() # Retrocompatibility again
    mask = sparse2dense(cv2.imread(path_mask, cv2.cv.CV_LOAD_IMAGE_GRAYSCALE))
    image_src = mask

    # Garment Segmentation Stage
    # Note: in this application the mask is obtained directly from the point
    # cloud depth data

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

    # Convert pick and place points to robot frame
    change_frame = Transformer()

    # Load configuration data and configure
    with open(path_input_mesh + "-origin.txt", "r") as f:
        file_contents = f.readlines()
        image_origin = [ float(i) for i in file_contents[0].split(' ')]
    change_frame.add_image_params((image_origin[0], image_origin[1]), 0.005)

    kinfu_wrt_object = np.loadtxt(path_input_mesh+"-transform.txt")
    change_frame.add_kinfu_wrt_object_transform(kinfu_wrt_object)
    change_frame.add_kinfu_params(np.linalg.inv(kinfu_wrt_cam))
    change_frame.add_cam_wrt_root_transform(H_root_cam)

    # Transform points (debug edition)
    pick_point_abs, place_point_abs = change_frame.debug([pick_point, place_point])
    print pick_point_abs, place_point_abs
    from common.perception.Utils import points_to_file
    points_to_file([pick_point_abs, place_point_abs], os.path.join(os.path.split(path_input_mesh)[0],
                                                                        "pick_and_place.pcd"))

    pick_point_root, place_point_root = change_frame.root([pick_point, place_point])
    distance = np.sqrt((pick_point_root[0]-place_point_root[0])**2+
                       (pick_point_root[1]-place_point_root[1])**2)

    print "Target points are:\n\tPick: {}\n\tPlace: {}\n\tMax height: {}".format(
        pick_point_root, place_point_root, distance*0.4)
    ans = query_yes_no("Perform pick and place operation? (WARNING: this operation might be potentially destructive)",
                 default="no")

    if ans == 'yes':

        robot.set_units(linear='meters',angular='degrees')
        robot.pick_and_place(pick_point_root[0], pick_point_root[1],
                             place_point_root[0], place_point_root[1], distance*0.4)

    robot.close()