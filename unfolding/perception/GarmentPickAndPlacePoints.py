import itertools
import math
from operator import itemgetter

import cv2

from common.perception import LineTools, Superpixels


class GarmentPickAndPlacePoints:

    @staticmethod
    def calculate_unfold_paths(labeled_image, approximated_polygon):
        # Calculate highest point
        highest_points = [Superpixels.get_highest_point_with_superpixels(labeled_image)[::-1]]

        # Get contour midpoints
        polygon_segments = LineTools.contour_to_segments(approximated_polygon)
        polygon_midpoints = [LineTools.midpoint(start, end) for start, end in polygon_segments]


        # Get paths to traverse:
        candidate_paths = list(itertools.product(highest_points, polygon_midpoints))
        valid_paths = list(filter(lambda x: len(LineTools.seg_intersection_polygon(x, polygon_segments)) <= 1,
                                  candidate_paths))

        return valid_paths

    @staticmethod
    def calculate_bumpiness(labeled_image, unfold_paths):
        profiles = [[p for p in Superpixels.line_sampling(labeled_image, path[0], path[1], 1) if p != 255]
                    for path in unfold_paths ]

        bumpiness = [sum([abs(j-i) for i, j in zip(profile, profile[1:])])
                     for profile in profiles]
        return bumpiness

    @staticmethod
    def calculate_pick_and_place_points(labeled_image, unfold_paths, bumpiness):
        # Select direction with lower bumpiness
        _, unfold_direction = min(zip(bumpiness, unfold_paths), key=itemgetter(0))
        highest_point, contour_point = unfold_direction

        # Find contour of highest region (lowest depth value from the camera)
        highest_region = Superpixels.get_highest_superpixel(labeled_image)
        highest_region_contours, dummy = cv2.findContours(highest_region.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        highest_region_contour = max(highest_region_contours, key=cv2.contourArea)

        # Find intersection with contour
        intersection = LineTools.line_intersection_polygon(unfold_direction,
                                                           LineTools.contour_to_segments(highest_region_contour))

        # Pick point is furthest from contour point
        pick_point = max(intersection, key=lambda x: math.hypot(contour_point[0]-x[0], contour_point[1]-x[1]))
        pick_point = (float(pick_point[0]), float(pick_point[1]))

        # Place point is reflection using contour point:
        place_point = (2*contour_point[0] - pick_point[0], 2*contour_point[1] - pick_point[1])

        return pick_point, place_point


