import Superpixels
from ClothContour import ClothContour

class GarmentPickAndPlacePoints:

    @staticmethod
    def calculate_unfold_paths(depth_map, labeled_image, approximated_polygon):
        # Calculate median value of each region
        average_regions = Superpixels.get_average_regions(depth_map, labeled_image)
        highest_points = [Superpixels.get_highest_point_with_superpixels(average_regions)[::-1]]

        # Get contour midpoints
        cloth_contour = ClothContour(approximated_polygon)

        # Get paths to traverse:
        valid_paths = cloth_contour.get_valid_paths(highest_points)

        return valid_paths

    @staticmethod
    def calculate_bumpiness(depth_map, labeled_image, unfold_paths):
        average_regions = Superpixels.get_average_regions(depth_map, labeled_image)
        profiles = [[p for p in Superpixels.line_sampling(average_regions, path[0], path[1], 1) if p != 255]
                    for id, path in unfold_paths ]

        bumpiness = [sum([abs(j-i) for i, j in zip(profile, profile[1:])])
                     for profile in profiles]
        return bumpiness

    @staticmethod
    def calculate_pick_and_place_points(unfold_paths, bumpiness):
        import numpy as np
        all_line_data=[]
        for id, path in unfold_paths:
            if path:
                start = [p for p in path[0]]
                end = [p for p in path[1]]
                all_line_data.append([start, end])
        selected_directions=[all_line_data[ np.argmin(bumpiness) ]]
        # Here we would put other selection criteria for directions (i.e. combining them)
        final_extremes = selected_directions[0]

    ############ SEGMENT EXTENDER
        # VARIABLE FOR FINAL POINTS (ONE IN SUPERPIXEL BORDER, THE OTHER IN IMAGE BORDER)
        extended_final_extremes = []

        # PROTOLINE

        from intersect import perp, line_intersect

        id_segment = edges_to_profiles[np.argmin(bumpiness)]

        perp_vector = perp(np.array(contour_segments[id_segment][1])-np.array(contour_segments[id_segment][0]))
        intersect_fold_axis = line_intersect(np.array(contour_segments[id_segment][0]),
                                             np.array(contour_segments[id_segment][1]),
                                             np.array(final_extremes[0]),
                                             np.array(final_extremes[0])+perp_vector)


       # LONG LINE MASK
        # extending final_extremes a lot to be sure it cuts countour highest superpixel
        long_line = segment_extender_twosides(final_extremes[0][0], final_extremes[0][1],
                                intersect_fold_axis[0], intersect_fold_axis[1])

        # create line mask to posterior logical intersection with countour
        line_mask = np.zeros(mask.shape,np.uint8)
        cv2.line(line_mask, (int(long_line[0][0]), int(long_line[0][1])),
                 (int(long_line[1][0]), int(long_line[1][1])), 255)


       # INITIAL POINT (INSIDE SUPERPIXEL)
        # extracting countour highest superpixel
        highest_region_mask = Superpixels.get_highest_superpixel(avg)
        kernel = np.ones((5,5),np.uint8)
        highest_region_mask_dilate = cv2.dilate(highest_region_mask,kernel,iterations = 1)
        # only contour mask
        contour_superpixel_mask = highest_region_mask_dilate - highest_region_mask

        # intersection long line and superpixel mask contour
        intersection_img = np.logical_and( contour_superpixel_mask, line_mask )

        # extracting countour centers (equivalent to extract intersection points)
        (contours,_) =  cv2.findContours(img_as_ubyte(intersection_img),cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        contour_intersect_points=[]
        for i in range(len(contours)):
            contour_intersect_points.append(np.average(contours[i], axis=0))

        # deciding which one is the initial initial point
        if np.fabs(distance.euclidean(final_extremes[0],contour_intersect_points[0][0])) < \
        np.fabs(distance.euclidean(final_extremes[1],contour_intersect_points[0][0])):
            extended_final_extremes.append( (contour_intersect_points[0][0]).tolist() )
            print "option 1 for initial point"

        else:
            extended_final_extremes.append( (contour_intersect_points[1][0]).tolist()  )
            print "option 2 for initial point"

       # FINAL POINT (INSIDE IMAGE)
        # reusing countour image and dilating
        mask_dilate = cv2.dilate(mask,kernel,iterations = 1)

        # only contour mask
        contour_mask = mask_dilate - mask

        # intersection long line and image mask contour
        intersection_img_2 = np.logical_and( contour_mask, line_mask )

        # extracting countour centers (equivalent to extract intersection points)
        (contours2,_) =  cv2.findContours(img_as_ubyte(intersection_img_2),cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        contour_intersect_points2=[]

        for i in range(len(contours2)):
            contour_intersect_points2.append(np.average(contours2[i], axis=0))

        # deciding which one is the final point
        if np.fabs(distance.euclidean(final_extremes[0],contour_intersect_points2[0][0])) < \
        np.fabs(distance.euclidean(final_extremes[1],contour_intersect_points2[0][0])):
            extended_final_extremes.append( (contour_intersect_points2[1][0]).tolist() )
            print "option 1 for final point"

        else:
            extended_final_extremes.append( (contour_intersect_points2[0][0]).tolist()  )
            print "option 2 for final point"

        # extending the segment out of the garment
        traj = segment_extender(extended_final_extremes[0][0], extended_final_extremes[0][1],
                                extended_final_extremes[1][0], extended_final_extremes[1][1])

        return traj