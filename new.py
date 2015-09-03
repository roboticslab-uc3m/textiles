# -*- coding: utf-8 -*-
"""
Created on Wed Sep  2 11:42:35 2015

@author: smorante
"""
import matplotlib.pyplot as plt
import numpy as np
import cv2

from segmentation import get_coloured_item
from utils import load_data
from mpl_toolkits.mplot3d import Axes3D
from skimage.segmentation import  slic
import Superpixels
from ClothContour import ClothContour
from scipy import ndimage as ndi        
from skimage.morphology import watershed, disk
from skimage.filters import rank
from skimage.util import img_as_ubyte
from skimage.restoration import denoise_tv_chambolle
from skimage import exposure
from skimage.color import gray2rgb
from skimage.util import img_as_float


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
#   cv2.imshow("mask", mask)

    # Obtain garment contour
    approx = get_garment_contour(mask)

    # Print clothes contour
    contour_show = image.copy()
    cv2.drawContours(contour_show, [approx], -1, (0, 0,255))
    for point in approx:
        cv2.circle(contour_show, tuple(point[0]), 3, (0, 0, 255), 2)
#   cv2.imshow("contour", contour_show)

    # Load depth map
    depth_image = np.loadtxt(path_depth)
    masked_depth_image = np.where(mask[:,:,0]==255, depth_image.transpose(), 1000)
    print "Loaded depth image, dimensions: " + str(depth_image.shape)


    # Normalize depth map
    scaled_depth_map = masked_depth_image.copy()
    min_value = masked_depth_image[np.unravel_index(np.where(masked_depth_image == 0, 1000,masked_depth_image).argmin(), masked_depth_image.shape)]
    max_value = masked_depth_image[np.unravel_index(np.where(masked_depth_image == 1000, 0,masked_depth_image).argmax(), masked_depth_image.shape)]
    range_value = max_value-min_value
    print "Scaled depth image dimensions: " + str(scaled_depth_map.shape)
    print "Depth image range: (%d, %d) delta=%d" % (min_value, max_value, range_value)
    scaled_depth_map = np.where(scaled_depth_map != 1000, (scaled_depth_map - min_value) * (255/range_value), 255)
    scaled_depth_map = scaled_depth_map.astype(np.uint8)
#   plt.figure(2)
#   plt.imshow(scaled_depth_map)
#   cv2.imshow("scaled", scaled_depth_map)

###############

    edges = get_garment_main_lines(scaled_depth_map)
    sobel_x = cv2.Sobel(scaled_depth_map, cv2.CV_64F, 1, 0, ksize=-1)
    sobel_y = cv2.Sobel(scaled_depth_map, cv2.CV_64F, 0, 1, ksize=-1)
    edges2 = np.sqrt(sobel_x**2 + sobel_y**2)
    min_ed, max_ed = edges2[np.unravel_index(edges2.argmin(), edges2.shape)], edges2[np.unravel_index(edges2.argmax(), edges2.shape)]
    print min_ed, max_ed
    edges2 = np.where(edges2 >= 350, 255, 0)
    edges = np.uint8(np.absolute(edges2))
#   show_image("canny2", edges)
    interior_edges = cv2.bitwise_and(edges, cv2.morphologyEx(mask, cv2.MORPH_ERODE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))))
#   plt.figure()        
#   plt.imshow(interior_edges)

    # Get highest_points
    highest_points = [Superpixels.get_highest_point_with_superpixels(scaled_depth_map)[::-1]]

    # Get contour midpoints
    cloth_contour = ClothContour(approx)
    contour_segments, contour_midpoints = cloth_contour.segments, cloth_contour.midpoints

    # Get paths to traverse:
    candidate_paths = cloth_contour.get_candidate_paths(highest_points)
    valid_paths = cloth_contour.get_valid_paths(highest_points)

########################################
## DOS MANERAS DE HALLAR SUPERPIXELS: SLICS O WATERSHED
########################################

######## SLIC
# # Calculate SLIC image:
#    img_src = scaled_depth_map
#    img = gray2rgb(img_src)
#    img = img_as_float(img[::2, ::2])
#    segments_slic = slic(img, n_segments=250, compactness=10, sigma=1, min_size_factor=200)

####### WATERSHED 
  #  img_eq = exposure.equalize_hist(scaled_depth_map)
    image = img_as_ubyte(scaled_depth_map)
        
  # denoise image
    denoised = denoise_tv_chambolle(image, weight=0.05)
# denoised = rank.median(image, disk(5))

  # find continuous region (low gradient) --> markers
    markers = rank.gradient(denoised, disk(10)) < 25
 # markers = rank.gradient(denoised, disk(10)) < 10
        
    markers = ndi.label(markers)[0]

  #local gradient
    gradient = rank.gradient(denoised, disk(5))
        
  # process the watershed
    labels = watershed(gradient, markers)
#        
  # display results
    fig, axes = plt.subplots(2,3)       
    axes[0, 0].imshow(image, cmap=plt.cm.gray, interpolation='nearest')
    axes[0, 1].imshow(denoised, cmap=plt.cm.gray, interpolation='nearest')
    axes[0, 2].imshow(markers, cmap=plt.cm.spectral, interpolation='nearest')
    axes[1, 0].imshow(gradient, cmap=plt.cm.spectral, interpolation='nearest')
    axes[1, 1].imshow(labels, cmap=plt.cm.spectral, interpolation='nearest', alpha=.7)               
    plt.show()
####################################

    #avg = Superpixels.get_average_slic(img_src[::2, ::2], segments_slic)
    
    img_src= scaled_depth_map    
    avg = Superpixels.get_average_slic(img_src, labels)

     #profiles
    for id, path in valid_paths:
        if path:
            start = [p for p in path[0]]
            end = [p for p in path[1]]
            path_samples_avg = Superpixels.line_sampling(avg, start , end, 1)
            path_samples_src = Superpixels.line_sampling(img_src, start, end, 1)
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


    
    
    
    
    
    