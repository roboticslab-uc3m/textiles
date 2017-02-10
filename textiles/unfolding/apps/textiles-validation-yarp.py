import numpy as np
import yarp

from textiles.common.perception.yarp_cameras import get_cameras
from textiles.unfolding.perception.GarmentDepthMapClustering import GarmentDepthMapClustering
from textiles.unfolding.perception.GarmentPickAndPlacePoints import GarmentPickAndPlacePoints
from textiles.unfolding.perception.GarmentSegmentation import GarmentSegmentation

if __name__ == '__main__':
    with get_cameras("/OpenNI2/imageFrame:o", (640, 480),
                     "/OpenNI2/depthFrame:o", (640, 480)) as (rgb_camera, depth_camera):
        # Open port for output data
        out_port = yarp.Port()
        out_port.open("/Unfolding/pnpPoints:o")

        # Process input
        while True:
            # Read input data
            image_src = rgb_camera.get_image()
            depth_image = depth_camera.get_image()

            # Garment Segmentation Stage
            mask = GarmentSegmentation.background_subtraction(image_src)
            approximated_polygon = GarmentSegmentation.compute_approximated_polygon(mask)

            # Garment Depth Map Clustering Stage
            preprocessed_depth_image = GarmentDepthMapClustering.preprocess(depth_image.transpose(), mask)
            labeled_image = GarmentDepthMapClustering.cluster_similar_regions(preprocessed_depth_image)
            # Garment Pick and Place Points Stage
            try:
                unfold_paths = GarmentPickAndPlacePoints.calculate_unfold_paths(labeled_image, approximated_polygon)
                bumpiness = GarmentPickAndPlacePoints.calculate_bumpiness(labeled_image, unfold_paths)
                pick_point, place_point = GarmentPickAndPlacePoints.calculate_pick_and_place_points(labeled_image,
                                                                                                    unfold_paths,
                                                                                                    bumpiness)
            except ValueError as e:
                # print("\t[-] Exception ocurred!", e)
                continue
            else:
                print(pick_point, place_point)
                bottle = yarp.Bottle()
                bottle.addDouble(pick_point[0])
                bottle.addDouble(pick_point[1])
                pick_point_x, pick_point_y = pick_point.astype(np.uint8)
                bottle.addInt(int(depth_image[pick_point_y, pick_point_x]))
                bottle.addDouble(place_point[0])
                bottle.addDouble(place_point[1])
                out_port.write(bottle)
