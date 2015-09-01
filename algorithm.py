import cv2
import numpy as np
from matplotlib import pyplot as plt

from segmentation import get_coloured_item
from utils import load_data
from ClothContour import ClothContour


__author__ = 'def'

def show_image(text, image):
    cv2.imshow(text, image)
    #cv2.waitKey(200)

def process_mask(mask, kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))):
    """
        Input: binary mask, kernel size for closing
        Output: filtered contours contained in image
    """
    filtered_mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    contours, dummy = cv2.findContours(filtered_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    contours_filtered = []
    for contour in contours:
        if len(contour) > 20 and cv2.contourArea(contour) > 400:
            contours_filtered.append(contour)

    contours_filtered.sort(key=lambda x: cv2.contourArea(x))
    contours_filtered.reverse()

    return contours_filtered

def simplify_contour(contour):
    perimeter = cv2.arcLength(contour,True)
    approx = cv2.approxPolyDP(contour,0.12*perimeter,True)

    if len(approx) == 4 and cv2.isContourConvex(approx):
        return approx
    else:
        return None

def contour_center(contour):
    M = cv2.moments(contour)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    return (cx, cy)


def get_garment_contour(mask):
    # Get clothes contour:
    #ret, image_contours_src = cv2.threshold(image, 240, 255, cv2.THRESH_BINARY_INV)
    clothes_contours, dummy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    clothes_contour = clothes_contours[0]
    # Size filtering:
    maxArea = cv2.contourArea(clothes_contour)
    for contour in clothes_contours[1:]:
        currentArea = cv2.contourArea(contour)
        if currentArea > maxArea:
            maxArea = currentArea
            clothes_contour = contour

    # Simplify contour:
    perimeter = cv2.arcLength(clothes_contour, True)
    approx = cv2.approxPolyDP(clothes_contour, 0.010 * perimeter, True)
    return approx


def get_garment_main_lines(image):
    # Apply canny to find 'not cross' lines
    blur = cv2.GaussianBlur(image, (11, 11), 0)
    inverted = cv2.bitwise_not(blur)
    # show_image("gauss", inverted)
    edges = cv2.Canny(inverted, 80, 160, apertureSize=3)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    # show_image("canny", edges)
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=5)
    return edges


def get_highest_points(image, threshold=40):
    # Find highest zones
    ret, highest_zones = cv2.threshold(image, threshold, 255, cv2.THRESH_BINARY_INV)
    #highest_contours = process_mask(highest_zones)
    highest_contours, dummy = cv2.findContours(highest_zones.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    show_image("highest_zones", highest_zones)
    highest_points = []
    for contour in highest_contours:
        highest_points.append(contour_center(contour))

    return highest_points


def main():
    #image_paths = ['./data/robe01_1_fold.ppm', './data/robe01_2_fold.ppm', './data/sweater02_1_fold.ppm', './data/sweater02_2_fold.ppm', './data/tshirt01_1_fold.ppm',
    #               './data/tshirt01_2_fold.ppm','./data/polo01_1_fold.ppm', './data/polo01_2_fold.ppm', './data/dishcloth01_2_fold.ppm', './data/dishcloth01_1_fold.ppm']

    #depth_maps = ['./data/robe01_1_fold.mat', './data/robe01_2_fold.mat', './data/sweater02_1_fold.mat', './data/sweater02_2_fold.mat', './data/tshirt01_1_fold.mat',
    #               './data/tshirt01_2_fold.mat','./data/polo01_1_fold.mat', './data/polo01_2_fold.mat', './data/dishcloth01_2_fold.mat', './data/dishcloth01_1_fold.mat']

    image_paths, depth_maps = load_data('./data/20150625_2')

    for path_rgb, path_depth in zip(image_paths, depth_maps):
        # Load image
        image = cv2.imread(path_rgb)

        # Get mask
        mask = get_coloured_item(image)
        show_image("mask", mask)
        cv2.destroyAllWindows()

        # Obtain garment contour
        approx = get_garment_contour(mask)

        # Print clothes contour
        contour_show = image.copy()
        cv2.drawContours(contour_show, [approx], -1, (0, 0,255))
        for point in approx:
            cv2.circle(contour_show, tuple(point[0]), 3, (0, 0, 255), 2)
        show_image("contour", contour_show)

        # Load depth map
        depth_image = np.loadtxt(path_depth)
        masked_depth_image = np.where(mask[:,:,0]==255, depth_image.transpose(), 1000)
        # plt.figure(1)
        # plt.hist(masked_depth_image)

        # Normalize depth map
        scaled_depth_map = masked_depth_image.copy()
        min_value = masked_depth_image[np.unravel_index(np.where(masked_depth_image == 0, 1000,masked_depth_image).argmin(), masked_depth_image.shape)]
        max_value = masked_depth_image[np.unravel_index(np.where(masked_depth_image == 1000, 0,masked_depth_image).argmax(), masked_depth_image.shape)]
        range_value = max_value-min_value
        print "Depth image range: (%d, %d) delta=%d" % (min_value, max_value, range_value)
        scaled_depth_map = np.where(scaled_depth_map != 1000, (scaled_depth_map - min_value) * (255/range_value), 255)
        # scaled_depth_map -= min_value
        # scaled_depth_map *= 255/range_value
        scaled_depth_map = scaled_depth_map.astype(np.uint8)
        # plt.figure(2)
        # plt.imshow(scaled_depth_map.astype(np.uint8))
        show_image("scaled", scaled_depth_map)
        # cv2.imwrite(path_rgb+'-bw.png', scaled_depth_map)
        # plt.show()

        # Get 'not crossing' lines
        edges = get_garment_main_lines(scaled_depth_map)
        sobel_x = cv2.Sobel(scaled_depth_map, cv2.CV_64F, 1, 0, ksize=-1)
        sobel_y = cv2.Sobel(scaled_depth_map, cv2.CV_64F, 0, 1, ksize=-1)
        edges2 = np.sqrt(sobel_x**2 + sobel_y**2)
        min_ed, max_ed = edges2[np.unravel_index(edges2.argmin(), edges2.shape)], edges2[np.unravel_index(edges2.argmax(), edges2.shape)]
        print min_ed, max_ed
        edges2 = np.where(edges2 >= 350, 255, 0)
        edges = np.uint8(np.absolute(edges2))
        plt.figure(1)
        plt.hist(edges2, bins=10)
        plt.figure(2)
        plt.imshow(edges, cmap='hot')
        plt.show()
        # show_image("canny2", edges)
        interior_edges = cv2.bitwise_and(edges, cv2.morphologyEx(mask, cv2.MORPH_ERODE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))))
        show_image("good", interior_edges)

        # Get highest_points
        try:
            highest_points = get_highest_points(scaled_depth_map,255/range_value*2)
        except:
            pass

        # Get contour midpoints
        cloth_contour = ClothContour(approx)
        contour_segments, contour_midpoints = cloth_contour.segments, cloth_contour.midpoints

        # Get paths to traverse:
        candidate_paths = cloth_contour.get_candidate_paths(highest_points)

        valid_paths = cloth_contour.get_valid_paths(highest_points)

        # Print clothes contour
        hp_show = image.copy()
        cv2.drawContours(hp_show, [approx], -1, (0, 0,255))
        for point in approx:
            cv2.circle(hp_show, tuple(point[0]), 3, (0, 0, 255), 2)
        for point in highest_points:
            cv2.circle(hp_show, tuple(point), 3, (255, 0, 0), 2)
        for point in contour_midpoints:
            cv2.circle(hp_show, tuple(point), 3, (0, 255, 255), 2)
        for id, path in valid_paths:
            if path:
                cv2.line(hp_show, tuple(path[0]), tuple(path[1]), (0 , 255, 0))
        show_image("h point", hp_show)

        # cv2.waitKey(-1)
        cv2.destroyAllWindows()

        # Check collisions (somehow) between trajectories and not-crossing lines
        fold_edges = set()
        for id, path in valid_paths:
            if path:
                # Get path bounding box:
                start = np.array(path[0])
                end = np.array(path[1])

                top_left_corner = np.minimum(start, end)
                sides = np.abs(start-end)

                # Crop the roi from the original image
                margin = 5
                roi_top_left_corner = top_left_corner-margin*np.array([1,1])
                roi_sides = sides + margin*np.array([2,2])

                roi_top_left_corner = np.clip(roi_top_left_corner, np.array([0,0]), image.shape[0:1])
                roi_sides = np.clip(roi_sides, np.array([0,0]), image.shape[0:1])

                roi_img = interior_edges[roi_top_left_corner[1]:roi_top_left_corner[1]+roi_sides[1],
                                         roi_top_left_corner[0]:roi_top_left_corner[0]+roi_sides[0]]

                # Generate a image for the path:
                line_thickness = 3
                path_img = np.zeros(roi_img.shape, dtype=np.uint8)
                cv2.line(path_img, tuple(start-roi_top_left_corner), tuple(end-top_left_corner), (255), line_thickness)

                # Check if they intersect. If they don't, the edge is a candidate to be the folding edge
                if cv2.countNonZero(cv2.bitwise_and(roi_img, path_img)) == 0:
                    fold_edges.add(id)

        print 'Fold edge(s) id(s): %s' % str(list(fold_edges))
        fold_edges_show = image.copy()
        for id in list(fold_edges):
                cv2.line(fold_edges_show, tuple(contour_segments[id][0]), tuple(contour_segments[id][1]), (255, 0, 255), 3)
        show_image("fold edges", fold_edges_show)
        cv2.waitKey(-1)
        cv2.destroyAllWindows()


def main2():
    #image_paths = ['./data/robe01_1_fold.ppm', './data/robe01_2_fold.ppm', './data/sweater02_1_fold.ppm', './data/sweater02_2_fold.ppm', './data/tshirt01_1_fold.ppm',
    #               './data/tshirt01_2_fold.ppm','./data/polo01_1_fold.ppm', './data/polo01_2_fold.ppm', './data/dishcloth01_2_fold.ppm', './data/dishcloth01_1_fold.ppm']

    #depth_maps = ['./data/robe01_1_fold.mat', './data/robe01_2_fold.mat', './data/sweater02_1_fold.mat', './data/sweater02_2_fold.mat', './data/tshirt01_1_fold.mat',
    #               './data/tshirt01_2_fold.mat','./data/polo01_1_fold.mat', './data/polo01_2_fold.mat', './data/dishcloth01_2_fold.mat', './data/dishcloth01_1_fold.mat']

    import matplotlib.pyplot as plt
    import numpy as np


    from skimage.segmentation import  slic
    from skimage.segmentation import mark_boundaries
    from skimage.util import img_as_float
    from skimage import io
    from skimage.color import gray2rgb
    import glob, os
    from matplotlib.backends.backend_pdf import PdfPages

    import Superpixels

    image_paths, depth_maps = load_data('./data/20150625_2')

    for path_rgb, path_depth in zip(image_paths, depth_maps):
        # Load image
        image = cv2.imread(path_rgb)
        print "Loaded rgb image, dimensions: " + str(image.shape)

        # Get mask
        mask = get_coloured_item(image)
        show_image("mask", mask)
        cv2.destroyAllWindows()

        # Obtain garment contour
        approx = get_garment_contour(mask)

        # Print clothes contour
        contour_show = image.copy()
        cv2.drawContours(contour_show, [approx], -1, (0, 0,255))
        for point in approx:
            cv2.circle(contour_show, tuple(point[0]), 3, (0, 0, 255), 2)
        show_image("contour", contour_show)

        # Load depth map
        depth_image = np.loadtxt(path_depth)
        masked_depth_image = np.where(mask[:,:,0]==255, depth_image.transpose(), 1000)
        print "Loaded depth image, dimensions: " + str(depth_image.shape)
        # plt.figure(1)
        # plt.hist(masked_depth_image)

        # Normalize depth map
        scaled_depth_map = masked_depth_image.copy()
        min_value = masked_depth_image[np.unravel_index(np.where(masked_depth_image == 0, 1000,masked_depth_image).argmin(), masked_depth_image.shape)]
        max_value = masked_depth_image[np.unravel_index(np.where(masked_depth_image == 1000, 0,masked_depth_image).argmax(), masked_depth_image.shape)]
        range_value = max_value-min_value
        print "Scaled depth image dimensions: " + str(scaled_depth_map.shape)
        print "Depth image range: (%d, %d) delta=%d" % (min_value, max_value, range_value)
        scaled_depth_map = np.where(scaled_depth_map != 1000, (scaled_depth_map - min_value) * (255/range_value), 255)
        # scaled_depth_map -= min_value
        # scaled_depth_map *= 255/range_value
        scaled_depth_map = scaled_depth_map.astype(np.uint8)
        # plt.figure(2)
        # plt.imshow(scaled_depth_map.astype(np.uint8))
        show_image("scaled", scaled_depth_map)
        # cv2.imwrite(path_rgb+'-bw.png', scaled_depth_map)
        # plt.show()

        # Get 'not crossing' lines
        edges = get_garment_main_lines(scaled_depth_map)
        sobel_x = cv2.Sobel(scaled_depth_map, cv2.CV_64F, 1, 0, ksize=-1)
        sobel_y = cv2.Sobel(scaled_depth_map, cv2.CV_64F, 0, 1, ksize=-1)
        edges2 = np.sqrt(sobel_x**2 + sobel_y**2)
        min_ed, max_ed = edges2[np.unravel_index(edges2.argmin(), edges2.shape)], edges2[np.unravel_index(edges2.argmax(), edges2.shape)]
        print min_ed, max_ed
        edges2 = np.where(edges2 >= 350, 255, 0)
        edges = np.uint8(np.absolute(edges2))
        # show_image("canny2", edges)
        interior_edges = cv2.bitwise_and(edges, cv2.morphologyEx(mask, cv2.MORPH_ERODE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))))
        show_image("good", interior_edges)

        # Get highest_points
        highest_points = [Superpixels.get_highest_point_with_superpixels(scaled_depth_map)[::-1]]

        # Get contour midpoints
        cloth_contour = ClothContour(approx)
        contour_segments, contour_midpoints = cloth_contour.segments, cloth_contour.midpoints

        # Get paths to traverse:
        candidate_paths = cloth_contour.get_candidate_paths(highest_points)
        valid_paths = cloth_contour.get_valid_paths(highest_points)

        # Calculate SLIC image:
        img_src = scaled_depth_map
        img = gray2rgb(img_src)
        img = img_as_float(img[::2, ::2])
        segments_slic = slic(img, n_segments=250, compactness=10, sigma=1, min_size_factor=200)
        avg = Superpixels.get_average_slic(img_src[::2, ::2], segments_slic)

        pp = PdfPages(path_rgb+'.pdf')

        for id, path in valid_paths:
            if path:
                start = [p/2 for p in path[0]]
                end = [p/2 for p in path[1]]
                path_samples_avg = Superpixels.line_sampling(avg, start , end, 1)
                path_samples_src = Superpixels.line_sampling(img_src[::2, ::2], start, end, 1)
                points = Superpixels.line_sampling_points(start, end, 1)

                fig, ax = plt.subplots(1, 2)
                fig.set_size_inches(8, 3, forward=True)
                fig.subplots_adjust(0.06, 0.08, 0.97, 0.91, 0.15, 0.05)

                ax[0].set_title(str(id)+': sampled profiles')
                # ax[0].plot(path_samples_avg, 'b-', path_samples_src, 'r-')
                without_white = [p for p in path_samples_avg if p != 255]
                ax[0].bar(range(len(without_white)), without_white, 1)

                ax[1].set_title(str(id)+': sampling points')
                ax[1].imshow(avg, cmap=plt.cm.gray)
                ax[1].plot(points[0], points[1], 'b-')

                plt.savefig(pp, format='pdf')

                # Generate csv file
                csv_filepath = os.path.splitext(path_rgb)[0] +'superpx.csv'
                with open(csv_filepath, 'w') as f:
                    for data in enumerate(path_samples_avg):
                        f.write('%d,%d\n' % data)

                csv_filepath = os.path.splitext(path_rgb)[0]+'raw.csv'
                with open(csv_filepath, 'w') as f:
                    for data in enumerate(path_samples_src):
                        f.write('%d,%d\n' % data)

        pp.close()
        # plt.show()

if __name__ == "__main__":
    main2()