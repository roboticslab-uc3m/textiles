import math
from operator import itemgetter

import numpy as np
import cv2

from textiles.common.perception import LineTools, Superpixels
from textiles.unfolding.perception.GarmentPickAndPlacePoints import GarmentPickAndPlacePoints
from textiles.common.math import mirror


class GarmentMirrorPickAndPlacePoints(GarmentPickAndPlacePoints):
    @staticmethod
    def calculate_pick_and_place_points(labeled_image, unfold_paths, bumpiness, approximated_polygon=None):
        if approximated_polygon is None: # Approximated polygon is required for mirroring
            return None

        # Select direction with lower bumpiness
        _, unfold_direction = min(zip(bumpiness, unfold_paths), key=itemgetter(0))
        highest_point, contour_point = unfold_direction

        # Find contour of highest region (lowest depth value from the camera)
        highest_region = Superpixels.get_highest_superpixel(labeled_image)
        _, highest_region_contours, _ = cv2.findContours(highest_region.copy(), cv2.RETR_EXTERNAL,
                                                         cv2.CHAIN_APPROX_SIMPLE)
        highest_region_contour = max(highest_region_contours, key=cv2.contourArea)

        # Find intersection with contour
        intersection = LineTools.line_intersection_polygon(unfold_direction,
                                                           LineTools.contour_to_segments(highest_region_contour))

        # Pick point is furthest from contour point
        pick_point = max(intersection, key=lambda x: math.hypot(contour_point[0]-x[0], contour_point[1]-x[1]))
        pick_point = (float(pick_point[0]), float(pick_point[1]))

        # Find symmetry axis:
        polygon_segments = LineTools.contour_to_segments(approximated_polygon)
        polygon_midpoints = [(i, LineTools.midpoint(start, end)) for i, (start, end) in enumerate(polygon_segments)]
        index, midpoint = min(polygon_midpoints, key=lambda x: math.hypot(contour_point[0]-x[1][0], contour_point[1]-x[1][1]))
        axis = polygon_segments[index]
        v = np.array(pick_point)-np.array(axis[0])
        l = np.array(axis[1])-np.array(axis[0])
        place_point = mirror(v, l) + np.array(axis[0])
        place_point = place_point.tolist()

        return pick_point, place_point


