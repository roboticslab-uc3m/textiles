# -*- coding: utf-8 -*-
"""
Created on Wed Sep  2 11:42:35 2015

@author: smorante, def
"""
import matplotlib.pyplot as plt
import numpy as np
import cv2
from scipy import ndimage as ndi        
from skimage.morphology import watershed, disk
from skimage.filters import rank
from skimage.util import img_as_ubyte
from skimage.restoration import denoise_tv_chambolle
from skimage import exposure

# def
from utils import load_data
import Superpixels
from ClothContour import ClothContour

########################################################
########################################################

def get_coloured_item(image):
    # Convert to HSV color space
    image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
 
   # Threshold value and saturation (using Otsu for threshold selection)
    blur_s = cv2.GaussianBlur(image_hsv[:, :, 1],(5,5),0)
    ret, mask_s = cv2.threshold(blur_s, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
#    cv2.imshow("----", mask_s)
   
    blur_v = cv2.GaussianBlur(image_hsv[:, :, 2],(5,5),0) 
    ret, mask_v = cv2.threshold(blur_v, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    mask = cv2.bitwise_and(mask_s, mask_v)

    # Filter result using morphological operations (closing)
    kernel = np.ones((5,5),np.uint8)    
    filtered_mask_close = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=5)
    filtered_mask_open = cv2.morphologyEx(filtered_mask_close, cv2.MORPH_OPEN, kernel, iterations=8)

    return filtered_mask_open

def get_garment_contour(mask):
    # Get clothes contour:
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
    cv2.imshow("gauss", inverted)
    
    edges = cv2.Canny(inverted, 80, 160, apertureSize=3)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    # show_image("canny", edges)
    
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=5)
    return edges

def get_image_with_contours(image, mask):
    # Obtain garment contour
    approx = get_garment_contour(mask)

    # Print clothes contour
    contour_show = image.copy()
    cv2.drawContours(contour_show, [approx], -1, (0, 0,255))
    for point in approx:
        cv2.circle(contour_show, tuple(point[0]), 3, (0, 0, 255), 2)
#    cv2.imshow("contour", contour_show)
   
    return approx   
    
def normalize_1Channel_image(image):
    scaled_depth_map = image.copy()
    min_value = masked_depth_image[np.unravel_index(np.where(masked_depth_image == 0, 1000,masked_depth_image).argmin(), masked_depth_image.shape)]
    max_value = masked_depth_image[np.unravel_index(np.where(masked_depth_image == 1000, 0,masked_depth_image).argmax(), masked_depth_image.shape)]
    range_value = max_value-min_value
    print "Scaled depth image dimensions: " + str(scaled_depth_map.shape)
    print "Depth image range: (%d, %d) delta=%d" % (min_value, max_value, range_value)
    scaled_depth_map = np.where(scaled_depth_map != 1000, (scaled_depth_map - min_value) * (255/range_value), 255)
    scaled_depth_map = scaled_depth_map.astype(np.uint8)
#    plt.figure(2)
#    plt.imshow(scaled_depth_map)
#    cv2.imshow("scaled", scaled_depth_map)
    return scaled_depth_map
    
def auto_canny(image, sigma=0.33):
    ## http://www.pyimagesearch.com/2015/04/06/zero-parameter-automatic-canny-edge-detection-with-python-and-opencv/
	# compute the median of the single channel pixel intensities
	v = np.median(image)
 
	# apply automatic Canny edge detection using the computed median
	lower = int(max(0, (1.0 - sigma) * v))
	upper = int(min(255, (1.0 + sigma) * v))
	edged = cv2.Canny(image, lower, upper)
 
	# return the edged image
	return edged    
#####################################################################
#####################################################################
#####################################################################

image_paths, depth_maps = load_data('./data/20150625_single')

for path_rgb, path_depth in zip(image_paths, depth_maps):
    # Load image
    image = cv2.imread(path_rgb)
    print "Loaded rgb image, dimensions: " + str(image.shape)

    # Get mask
    mask = get_coloured_item(image)

    # Print clothes contour
    approx =  get_image_with_contours(image, mask)
    
    # Load depth map
    depth_image = np.loadtxt(path_depth)
    masked_depth_image = np.where(mask==255, depth_image.transpose(), 1000)
    print "Loaded depth image, dimensions: " + str(depth_image.shape)

    # Normalize depth map
    scaled_depth_map = normalize_1Channel_image(masked_depth_image)


######## HIGHEST POINT
    # Get highest_points
    highest_points = [Superpixels.get_highest_point_with_superpixels(scaled_depth_map)[::-1]]

    # Get contour midpoints
    cloth_contour = ClothContour(approx)
    contour_segments, contour_midpoints = cloth_contour.segments, cloth_contour.midpoints

    # Get paths to traverse:
    valid_paths = cloth_contour.get_valid_paths(highest_points)


####### WATERSHED 
    image = img_as_ubyte(scaled_depth_map)
          
    # denoise image
    denoised = denoise_tv_chambolle(image, weight=0.05)
    denoised_equalize= exposure.equalize_hist(denoised)  

    # find continuous region (low gradient) --> markers
    markers = rank.gradient(denoised_equalize, disk(25)) < 15 # 25,15  10,10
# 2 #   markers = auto_canny(img_as_ubyte(denoised_equalize), 5)
# 3 #    markers = cv2.Canny(img_as_ubyte(denoised_equalize), 10,200)

    markers = ndi.label(markers)[0]

    # local gradient
    gradient = rank.gradient(denoised, disk(5))

    # labels
    labels = watershed(gradient, markers)
  
    # display results
    fig, axes = plt.subplots(2,3)       
    axes[0, 0].imshow(image, cmap=plt.cm.gray, interpolation='nearest')
    axes[0, 1].imshow(denoised, cmap=plt.cm.gray, interpolation='nearest')
    axes[0, 2].imshow(markers, cmap=plt.cm.spectral, interpolation='nearest')
    axes[1, 0].imshow(gradient, cmap=plt.cm.spectral, interpolation='nearest')
    axes[1, 1].imshow(labels, cmap=plt.cm.spectral, interpolation='nearest', alpha=.7)               
    plt.show()


######### PATHS
#    # calculate heights paths
#    img_src= scaled_depth_map    
#    avg = Superpixels.get_average_regions(img_src, labels)
#
#    # profiles
#    for id, path in valid_paths:
#        if path:
#            start = [p for p in path[0]]
#            end = [p for p in path[1]]
#            path_samples_avg = Superpixels.line_sampling(avg, start , end, 1)
#            path_samples_src = Superpixels.line_sampling(img_src, start, end, 1)
#            points = Superpixels.line_sampling_points(start, end, 1)
#
#            fig, ax = plt.subplots(1, 2)
#            fig.set_size_inches(8, 3, forward=True)
#            fig.subplots_adjust(0.06, 0.08, 0.97, 0.91, 0.15, 0.05)
#
#            ax[0].set_title(str(id)+': sampled profiles')
#           # ax[0].plot(path_samples_avg, 'b-', path_samples_src, 'r-')
#            without_white = [p for p in path_samples_avg if p != 255]
#            ax[0].bar(range(len(without_white)), without_white, 1)
#
#            ax[1].set_title(str(id)+': sampling points')
#            ax[1].imshow(avg, cmap=plt.cm.gray)
#            ax[1].plot(points[0], points[1], 'b-')
