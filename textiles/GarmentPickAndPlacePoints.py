from operator import itemgetter
import cv2

import Superpixels
from ClothContour import ClothContour
import LineTools

class GarmentPickAndPlacePoints:

    @staticmethod
    def calculate_unfold_paths(labeled_image, approximated_polygon):
        # Calculate highest point
        highest_points = [Superpixels.get_highest_point_with_superpixels(labeled_image)[::-1]]

        # Get contour midpoints
        cloth_contour = ClothContour(approximated_polygon)

        # Get paths to traverse:
        valid_paths = [path for id, path in cloth_contour.get_valid_paths(highest_points) if path]
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
        print _, unfold_direction

        # Find contour of highest region (lowest depth value from the camera)
        highest_region = Superpixels.get_highest_superpixel(labeled_image)
        highest_region_contours, dummy = cv2.findContours(highest_region.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        highest_region_contour = max(highest_region_contours, key=cv2.contourArea)

        # Find intersection with contour
        intersection = LineTools.line_intersection_polygon(unfold_direction,
                                                           LineTools.contour_to_segments(highest_region_contour))
        if intersection:
            pick, place = intersection
            return pick, place
        else:
            raise BaseException('Bad implementation')


