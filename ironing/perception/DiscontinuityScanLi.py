# coding=utf-8

import os
from skimage import io
from skimage import img_as_ubyte
import numpy as np
import matplotlib.pyplot as plt
import cv2 # SIFT, SVM not in skimage
import begin
import logging
from common.math import normalize_array
from common.perception.Features import save_SIFT

"""
Discontinuity Scan Li
---------------------------------------------------------------------
Discontinuity scan as described in Yinxiao Li's Thesis and in article:
 * Y. Li, X. Hu, D. Xu, Y. Yue, E. Grinspun, and P. Allen, “Multi-
Sensor Surface Analysis for Robotic Ironing,” in IEEE International
Conference on Robotics and Automation (ICRA), Stockholm, 2016.

Finds wrinkles on garments based on rgb images taken with two different
perpendicular illumination sources.
"""

def load_roi_from_file(filename):
    """
    Loads region of interest from a file. The file format is the following:
    start_x start_y
    end_x end_y
    :param filename: Path to the file
    :return: a region of interest described as two points -> (start_x, start_y), (end_x, end_y)
    """

    with open(filename, 'r') as f:
        lines = f.readlines()
        (start_x, start_y), (end_x, end_y) = [map(int, line.split(' ')) for line in lines]

    return (start_x, start_y), (end_x, end_y)

def crop_roi(roi_rect, image):
    """
    Uses a ROI rectangle to crop a Region Of Interest of the image
    """
    return image[roi_rect[0][1]:roi_rect[1][1], roi_rect[0][0]:roi_rect[1][0]]

class DiscontinuityScanLi(object):
    image_wrinkles_name_pattern = "garment-{:02d}-image-{:02d}.ppm"
    image_reference_name_pattern = "garment-{:02d}-imageref-{:02d}.ppm"
    image_roi_name_pattern = "garment-{:02d}-roi.txt"

    def __init__(self):
        # General images
        self.image_wrinkles_1 = None
        self.image_wrinkles_2 = None
        self.image_ref_1 = None
        self.image_ref_2 = None
        # Normalized images
        self.norm = None
        self.norm_1 = None
        self.norm_2 = None
        # Region of interest
        self.roi_rect = None
        # SIFT descriptors
        self.keypoints = None
        self.descriptors = None

    def load_images(self, image_folder, image_id=0, use_roi=True):
        image_folder = os.path.abspath(os.path.expanduser(image_folder))

        self.image_wrinkles_1 = io.imread(os.path.join(image_folder, self.image_wrinkles_name_pattern.format(image_id, 1)), as_grey=True)
        self.image_wrinkles_2 = io.imread(os.path.join(image_folder, self.image_wrinkles_name_pattern.format(image_id, 2)), as_grey=True)

        self.image_ref_1 = io.imread(os.path.join(image_folder, self.image_reference_name_pattern.format(image_id, 1)), as_grey=True)
        self.image_ref_2 = io.imread(os.path.join(image_folder, self.image_reference_name_pattern.format(image_id, 2)), as_grey=True)

        if use_roi:
            try:
                self.roi_rect = load_roi_from_file(os.path.join(image_folder, self.image_roi_name_pattern.format(image_id)))

                self.image_wrinkles_1 = crop_roi(self.roi_rect, self.image_wrinkles_1)
                self.image_wrinkles_2 = crop_roi(self.roi_rect, self.image_wrinkles_2)
                self.image_ref_1 = crop_roi(self.roi_rect, self.image_ref_1)
                self.image_ref_2 = crop_roi(self.roi_rect, self.image_ref_2)
            except FileNotFoundError:
                logging.warning("Could not load ROI, file does not exist")


    def normalize_images(self):
        """
        Apply normalization method as described in Li's algorithm
        """
        self.norm_1 = self.image_wrinkles_1 / self.image_ref_1
        self.norm_2 = self.image_wrinkles_2 / self.image_ref_2
        self.norm = np.sqrt(np.power(self.norm_1,2)+np.power(self.norm_2,2))

    def compute_SIFT(self, labels=None):
        """
        Computes SIFT features using OpenCV
        :param labels: if provided, the labeled image is used to label the class_id member of each keypoint
        """
        # Compute SIFT features
        sift = cv2.xfeatures2d.SIFT_create()
        self.keypoints, self.descriptors = sift.detectAndCompute(img_as_ubyte(normalize_array(self.norm)),None)
        logging.debug("Detected {} keypoints".format(len(self.keypoints)))

        # Debug stuff
        # img = cv2.drawKeypoints(img_as_ubyte(normalize_array(self.norm)),keypoints, None)
        # cv2.imwrite('sift_keypoints.jpg',img)

        # Label keypoints
        if labels is not None:
            for kp in self.keypoints:
                y, x = map(int, kp.pt)
                kp.class_id = int(labels[x, y])

            for kp in self.keypoints:
                logging.debug("Keypoint at {}, class {}".format(kp.pt, kp.class_id))



    def plot_input_images(self, colormap=plt.cm.viridis):
        """
        Plot all 4 input images in a single figure
        """
        f, ax = plt.subplots(2, 2)
        ax[0][0].imshow(self.image_wrinkles_1, cmap=colormap)
        ax[0][1].imshow(self.image_wrinkles_2, cmap=colormap)
        ax[1][0].imshow(self.image_ref_1, cmap=colormap)
        ax[1][1].imshow(self.image_ref_2, cmap=colormap)
        plt.show()

    def plot_normalized_images(self, colormap=plt.cm.viridis):
        f, ax = plt.subplots(1, 3)
        ax[0].imshow(self.norm_1, cmap=colormap)
        ax[1].imshow(self.norm_2, cmap=colormap)
        ax[2].imshow(self.norm , cmap=colormap)
        plt.show()


def process_images(image_folder, image_id, display_results=False):
    logging.info("Processing image {}".format(image_id))
    discontinuity_scanner = DiscontinuityScanLi()

    logging.debug("\tLoading images...")
    discontinuity_scanner.load_images(image_folder, image_id=image_id, use_roi=True)

    logging.info("\tNormalizing images...")
    discontinuity_scanner.normalize_images()

    if display_results:
        discontinuity_scanner.plot_input_images()
        discontinuity_scanner.plot_normalized_images()

    return discontinuity_scanner


@begin.subcommand
@begin.convert(_automatic=True)
def generate_dataset(num_images: 'Number of images in image folder' = 0, display_results: 'Show feedback of the process' = False,
          *image_folder):
    """
    Generates a image dataset from the normalized images
    """
    image_output_name_pattern = "garment-{:02d}-out.png"
    image_folder = map(lambda x: os.path.abspath(os.path.expanduser(x)), image_folder)
    image_folder = list(image_folder)[0]
    logging.info("Loading images from {}".format(image_folder))
    for i in range(1, num_images+1): # For each sample image
        discontinuity_scanner = process_images(image_folder, i, display_results)
        logging.info("\tGenerating dataset")
        colormap = plt.cm.viridis # or gray, inferno, etc
        io.imsave(os.path.join(image_folder, image_output_name_pattern.format(i)), colormap(normalize_array(discontinuity_scanner.norm)))


@begin.subcommand
@begin.convert(_automatic=True)
def compute_sift(num_images: 'Number of images in image folder' = 0, display_results: 'Show feedback of the process' = False,
          *image_folder):
    """
    Computes the SIFT descriptors of the labeled images
    """
    image_labels_name_pattern = "garment-{:02d}-labels.png"
    sift_features_name_pattern = "garment-{:02d}-sift.npz"
    sift_features_class_name_pattern = "garment-{:02d}-sift-classes.npz"
    image_folder = map(lambda x: os.path.abspath(os.path.expanduser(x)), image_folder)
    image_folder = list(image_folder)[0]
    logging.info("Loading images from {}".format(image_folder))

    for i in range(1, num_images+1): # For each sample image
        discontinuity_scanner = process_images(image_folder, i, display_results)
        logging.info("\tLoading labels...")
        try:
            labels = io.imread(os.path.join(image_folder, image_labels_name_pattern.format(i)), as_grey=True)
            labels=np.where(labels > 0.5, 1, 0) # Temporary fix for gimp creating images with weird values.
        except FileNotFoundError:
            logging.info("\t\tLabels not found, skipping them!")
            labels = None

        logging.info("\tComputing SIFT features...")
        discontinuity_scanner.compute_SIFT(labels=labels)
        if display_results:
            keypoints = discontinuity_scanner.keypoints
            plt.imshow(discontinuity_scanner.norm, cmap=plt.cm.viridis)
            xlims, ylims = plt.xlim(), plt.ylim()
            for k in keypoints:
                plt.plot(k.pt[0], k.pt[1], 'r*' if k.class_id == 0 else 'bo')
            plt.xlim(xlims[0], xlims[1])
            plt.ylim(ylims[0], ylims[1])
            plt.show()


        logging.info("\tSaving SIFT features...")
        save_SIFT(os.path.join(image_folder, sift_features_name_pattern.format(i)),
                  discontinuity_scanner.keypoints, discontinuity_scanner.descriptors,
                  class_id_filename = os.path.join(image_folder, sift_features_class_name_pattern.format(i)))

@begin.subcommand
@begin.convert(_automatic=True)
def train_svm(num_images: 'Number of images in image folder' = 0, *image_folder):
    """
    Loads SIFT data from the specified path and trains a SVM with them
    """
    sift_features_name_pattern = "garment-{:02d}-sift.npz"
    sift_features_class_name_pattern = "garment-{:02d}-sift-classes.npz"
    svm_data_name_pattern = "garment-svm.dat"
    image_folder = map(lambda x: os.path.abspath(os.path.expanduser(x)), image_folder)
    image_folder = list(image_folder)[0]

    logging.info("Started training SVM...")
    des = np.empty((0, 4+128)) # point x, point y, feature scale, feature orientation + descritor size (128 cols)
    y = np.empty((0, 1))
    for i in range(1, num_images+1): # For each sample image
        logging.info("\tLoading data from image {}...".format(i))
        des = np.vstack((des, np.load(os.path.join(image_folder, sift_features_name_pattern.format(i)))['descriptors']))
        new_y = np.load(os.path.join(image_folder, sift_features_class_name_pattern.format(i)))['y']
        y = np.vstack((y, np.reshape(new_y, (new_y.shape[0], 1))))
    if des.shape[0] != y.shape[0]:
        logging.error("Descriptor matrix does not match classes matrix")
        return -1

    _ , counts = np.unique(y, return_counts=True)
    logging.info("Loaded {} examples, {} negative and {} positive".format(des.shape[0], counts[0], counts[1]))

    logging.info("Training SVM with examples...")
    svm_params = dict( kernel_type = cv2.ml.SVM_LINEAR, svm_type = cv2.ml.SVM_C_SVC, C=2.67, gamma=5.383 ) # Need to find a way to set this
    svm = cv2.ml.SVM_create()
    svm.train(np.float32(des[:, 4:]), cv2.ml.ROW_SAMPLE, np.int32(y))
    svm.save(os.path.join(image_folder, svm_data_name_pattern))
    return 0

@begin.subcommand
@begin.convert(_automatic=True)
def predict(svm_datafile: 'File storing the SVM parameters' = '', image_id: 'Id of the image to use for prediction' = 0,
        display_results: 'Show feedback of the results' = False, *image_folder):
    sift_features_name_pattern = "garment-{:02d}-sift.npz"
    image_folder = map(lambda x: os.path.abspath(os.path.expanduser(x)), image_folder)
    image_folder = list(image_folder)[0]

    logging.info("Loading trained SVM data...")
    try:
        svm = cv2.ml.SVM_load(os.path.abspath(os.path.expanduser(svm_datafile)))
    except AttributeError:
        logging.error("\tYour OpenCV version does not support loading SVM parameters")
        return -1

    logging.info("Loading SIFT descriptors...")
    des =  np.load(os.path.join(image_folder, sift_features_name_pattern.format(image_id)))['descriptors']

    logging.info("Predicting...")
    retval, result = svm.predict(np.float32(des[:,4:]))
    values , counts = np.unique(result, return_counts=True)
    try:
        logging.info("\tPredicted {} points, {} negative and {} positive".format(result.shape[0], counts[0], counts[1]))
    except IndexError:
        logging.info("\tPredicted {} points, {} negative and {} positive".format(result.shape[0], 0 if 0 not in values else counts[0],
                                                                                 0 if 1 not in values else counts[0]))

    # Visual feedback
    if display_results:
        # Load output image
        image_output_name_pattern = "garment-{:02d}-out.png"
        image = io.imread(os.path.join(image_folder, image_output_name_pattern.format(image_id)))

        # Load descriptors
        keypoints = des[:, :2] # (x, y) are the first columns of the matrix
        plt.imshow(image)
        xlims, ylims = plt.xlim(), plt.ylim() # Save plot dimensions to restore them
        for example, prediction in zip(keypoints, result):
            plt.plot(example[0], example[1], 'r*' if prediction == 0 else 'bo')
        plt.xlim(xlims[0], xlims[1]) # Restore x axis
        plt.ylim(ylims[0], ylims[1]) # Restore y axis
        plt.show()

    # If labels exist, maybe check out good is the prediction with them here

    # And here we compute the hough transform maybe?

@begin.start(auto_convert=True)
@begin.logging
def main():
    pass