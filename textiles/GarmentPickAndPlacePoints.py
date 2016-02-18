import Superpixels
from ClothContour import ClothContour

class GarmentPickAndPlacePoints:

    @staticmethod
    def calculate_unfold_paths(labeled_image, approximated_polygon):
        # Calculate highest point
        highest_points = [Superpixels.get_highest_point_with_superpixels(labeled_image)[::-1]]

        # Get contour midpoints
        cloth_contour = ClothContour(approximated_polygon)

        # Get paths to traverse:
        valid_paths = cloth_contour.get_valid_paths(highest_points)

        return valid_paths

    @staticmethod
    def calculate_bumpiness(labeled_image, unfold_paths):
        profiles = [[p for p in Superpixels.line_sampling(labeled_image, path[0], path[1], 1) if p != 255]
                    for id, path in unfold_paths ]

        bumpiness = [sum([abs(j-i) for i, j in zip(profile, profile[1:])])
                     for profile in profiles]
        return bumpiness

    @staticmethod
    def calculate_pick_and_place_points(labeled_image, unfold_paths, bumpiness):
        return None, None


