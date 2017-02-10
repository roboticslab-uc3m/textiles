# coding=utf-8

import os
import logging

import numpy as np
import matplotlib.pyplot as plt
from skimage import io
from skimage import img_as_ubyte
from skimage.transform import hough_line, hough_line_peaks
import cv2  # SIFT, SVM not in skimage

from textiles.common.math import normalize
from textiles.common.perception.Features import save_SIFT
from textiles.common.perception.roi import load_roi_from_file, crop_roi

logger = logging.getLogger(__name__)

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


class DiscontinuityScanLi(object):
    image_wrinkles_name_pattern = "garment-{:02d}-image-{:02d}.ppm"
    image_reference_name_pattern = "garment-{:02d}-imageref-{:02d}.ppm"
    image_roi_name_pattern = "garment-{:02d}-roi.txt"
    image_labels_name_pattern = "garment-{:02d}-labels.png"
    sift_features_name_pattern = "garment-{:02d}-sift.npz"
    sift_features_class_name_pattern = "garment-{:02d}-sift-classes.npz"
    svm_data_name_pattern = "garment-svm.dat"

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
        # SVM
        self.svm = None

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
                logger.warning("Could not load ROI, file does not exist")

    def normalize_images(self):
        """
        Apply normalization method as described in Li's algorithm
        """
        self.norm_1 = self.image_wrinkles_1 / self.image_ref_1
        self.norm_2 = self.image_wrinkles_2 / self.image_ref_2
        self.norm = np.sqrt(np.power(self.norm_1, 2)+np.power(self.norm_2, 2))

    def compute_SIFT(self, labels=None):
        """
        Computes SIFT features using OpenCV
        :param labels: if provided, the labeled image is used to label the class_id member of each keypoint
        """
        # Compute SIFT features
        sift = cv2.xfeatures2d.SIFT_create()
        self.keypoints, self.descriptors = sift.detectAndCompute(img_as_ubyte(normalize(self.norm)), None)
        logger.debug("Detected {} keypoints".format(len(self.keypoints)))

        # Debug stuff
        # img = cv2.drawKeypoints(img_as_ubyte(normalize_array(self.norm)),keypoints, None)
        # cv2.imwrite('sift_keypoints.jpg',img)

        # Label keypoints
        if labels is not None:
            for kp in self.keypoints:
                y, x = map(int, kp.pt)
                kp.class_id = int(labels[x, y])

            for kp in self.keypoints:
                logger.debug("Keypoint at {}, class {}".format(kp.pt, kp.class_id))

    def load_svm(self, svm_folder):
        """
        Loads the SVM parameters from a file named as the attribute self.svm_data_name_pattern in the specified folder
        """
        svm_folder = os.path.abspath(os.path.expanduser(svm_folder))

        try:
            self.svm = cv2.ml.SVM_load(os.path.join(svm_folder, self.svm_data_name_pattern))
        except AttributeError:
            logger.error("\tYour OpenCV version does not support loading SVM parameters")
            raise AttributeError("OpenCV version does not support loading SVM parameters")

    def run(self, debug=False):
        self.normalize_images()
        self.compute_SIFT()

        # Prediction
        retval, result = self.svm.predict(np.float32(self.descriptors[:,4:]))
        values, counts = np.unique(result, return_counts=True)
        try:
            logger.info("\tPredicted {} points, {} negative and {} positive".format(result.shape[0], counts[0], counts[1]))
        except IndexError:
            logger.info("\tPredicted {} points, {} negative and {} positive".format(result.shape[0],
                                                                                    0 if 0 not in values else counts[0],
                                                                                    0 if 1 not in values else counts[0]))
        # Visual feedback
        if debug:
            # Load descriptors
            keypoints = self.descriptors[:, :2]  # (x, y) are the first columns of the matrix
            plt.imshow(self.norm)
            xlims, ylims = plt.xlim(), plt.ylim()  # Save plot dimensions to restore them
            for example, prediction in zip(keypoints, result):
                plt.plot(example[0], example[1], 'r*' if prediction == 0 else 'bo')
            plt.xlim(xlims[0], xlims[1])  # Restore x axis
            plt.ylim(ylims[0], ylims[1])  # Restore y axis
            plt.show()

        # Hough transform
        result_image = np.zeros((self.norm.shape[0], self.norm.shape[1]))
        for example, prediction in zip(self.keypoints, result):
            if prediction == 1:
                result_image[int(example[1]), int(example[0])] = 255

        h, theta, d = hough_line(result_image)

        # Generate image from Hough transform lines
        hough_lines_image = np.zeros((self.norm.shape[0], self.norm.shape[1]))
        for _, angle, dist in zip(*hough_line_peaks(h, theta, d)):
            y0 = (dist - 0 * np.cos(angle)) / np.sin(angle)
            y1 = (dist - self.norm.shape[1] * np.cos(angle)) / np.sin(angle)
            cv2.line(hough_lines_image, (0, self.norm.shape[1]), (y0, y1), 0)

        if debug:
            fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(8,4))

            ax1.imshow(result_image, cmap=plt.cm.gray)
            ax1.set_title('Input image')
            ax1.set_axis_off()

            ax2.imshow(np.log(1 + h),
                         extent=[np.rad2deg(theta[-1]), np.rad2deg(theta[0]),
                                 d[-1], d[0]],
                         cmap=plt.cm.gray, aspect=1/1.5)
            ax2.set_title('Hough transform')
            ax2.set_xlabel('Angles (degrees)')
            ax2.set_ylabel('Distance (pixels)')
            ax2.axis('image')

            ax3.imshow(result_image, cmap=plt.cm.gray)
            rows, cols = result_image.shape
            for _, angle, dist in zip(*hough_line_peaks(h, theta, d)):
                y0 = (dist - 0 * np.cos(angle)) / np.sin(angle)
                y1 = (dist - cols * np.cos(angle)) / np.sin(angle)
                ax3.plot((0, cols), (y0, y1), '-r')
            ax3.axis((0, cols, rows, 0))
            ax3.set_title('Detected lines')
            ax3.set_axis_off()

            plt.show()

        return result, hough_lines_image

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
        ax[2].imshow(self.norm, cmap=colormap)
        plt.show()


def process_images(image_folder, image_id, display_results=False):
    logger.info("Processing image {}".format(image_id))
    discontinuity_scanner = DiscontinuityScanLi()

    logger.debug("\tLoading images...")
    discontinuity_scanner.load_images(image_folder, image_id=image_id, use_roi=True)

    logger.info("\tNormalizing images...")
    discontinuity_scanner.normalize_images()

    if display_results:
        discontinuity_scanner.plot_input_images()
        discontinuity_scanner.plot_normalized_images()

    return discontinuity_scanner


def generate_dataset(num_images: 'Number of images in image folder' = 0, display_results: 'Show feedback of the process' = False,
          image_folder=list()):
    """
    Generates a image dataset from the normalized images
    """
    image_output_name_pattern = "garment-{:02d}-out.png"
    image_folder = map(lambda x: os.path.abspath(os.path.expanduser(x)), image_folder)
    image_folder = list(image_folder)[0]
    logger.info("Loading images from {}".format(image_folder))
    for i in range(1, num_images+1):  # For each sample image
        discontinuity_scanner = process_images(image_folder, i, display_results)
        logger.info("\tGenerating dataset")
        colormap = plt.cm.viridis  # or gray, inferno, etc
        io.imsave(os.path.join(image_folder, image_output_name_pattern.format(i)), colormap(normalize(discontinuity_scanner.norm)))


def compute_sift(num_images: 'Number of images in image folder' = 0, display_results: 'Show feedback of the process' = False,
          image_folder=list()):
    """
    Computes the SIFT descriptors of the labeled images
    """
    image_folder = map(lambda x: os.path.abspath(os.path.expanduser(x)), image_folder)
    image_folder = list(image_folder)[0]
    logger.info("Loading images from {}".format(image_folder))

    for i in range(1, num_images+1): # For each sample image
        discontinuity_scanner = process_images(image_folder, i, display_results)
        logger.info("\tLoading labels...")
        try:
            labels = io.imread(os.path.join(image_folder, DiscontinuityScanLi.image_labels_name_pattern.format(i)),
                               as_grey=True)
            labels=np.where(labels > 0.5, 1, 0)  # Temporary fix for gimp creating images with weird values.
        except FileNotFoundError:
            logger.info("\t\tLabels not found, skipping them!")
            labels = None

        logger.info("\tComputing SIFT features...")
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

        logger.info("\tSaving SIFT features...")
        save_SIFT(os.path.join(image_folder, DiscontinuityScanLi.sift_features_name_pattern.format(i)),
                  discontinuity_scanner.keypoints, discontinuity_scanner.descriptors,
                  class_id_filename=os.path.join(image_folder, DiscontinuityScanLi.sift_features_class_name_pattern.format(i)))


def train_svm(num_images: 'Number of images in image folder' = 0, image_folder=list()):
    """
    Loads SIFT data from the specified path and trains a SVM with them
    """
    image_folder = map(lambda x: os.path.abspath(os.path.expanduser(x)), image_folder)
    image_folder = list(image_folder)[0]

    logger.info("Started training SVM...")
    des = np.empty((0, 4+128))  # point x, point y, feature scale, feature orientation + descritor size (128 cols)
    y = np.empty((0, 1))
    for i in range(1, num_images+1):  # For each sample image
        logger.info("\tLoading data from image {}...".format(i))
        des = np.vstack((des, np.load(os.path.join(image_folder,
                                                   DiscontinuityScanLi.sift_features_name_pattern.format(i)))['descriptors']))
        new_y = np.load(os.path.join(image_folder, DiscontinuityScanLi.sift_features_class_name_pattern.format(i)))['y']
        y = np.vstack((y, np.reshape(new_y, (new_y.shape[0], 1))))
    if des.shape[0] != y.shape[0]:
        logger.error("Descriptor matrix does not match classes matrix")
        return -1

    _ , counts = np.unique(y, return_counts=True)
    logger.info("Loaded {} examples, {} negative and {} positive".format(des.shape[0], counts[0], counts[1]))

    logger.info("Training SVM with examples...")
    svm_params = dict( kernel_type=cv2.ml.SVM_LINEAR, svm_type=cv2.ml.SVM_C_SVC, C=2.67, gamma=5.383)  # Need to find a way to set this
    svm = cv2.ml.SVM_create()
    svm.train(np.float32(des[:, 4:]), cv2.ml.ROW_SAMPLE, np.int32(y))
    svm.save(os.path.join(image_folder, DiscontinuityScanLi.svm_data_name_pattern))
    return 0


def predict(image_id: 'Id of the image to use for prediction' = 0, display_results: 'Show feedback of the results' = False,
            image_folder=list()):
    image_folder = map(lambda x: os.path.abspath(os.path.expanduser(x)), image_folder)
    image_folder = list(image_folder)[0]

    # Create scanner and load images
    discontinuity_scanner = process_images(image_folder, image_id, display_results)

    logger.info("Loading trained SVM data...")
    discontinuity_scanner.load_svm(image_folder)

    logger.info("Predicting...")
    result, _ = discontinuity_scanner.run(display_results)

    # If labels exist, check out good is the prediction with them here
    try:
        labels = np.load(os.path.join(image_folder,
                                      DiscontinuityScanLi.sift_features_class_name_pattern.format(image_id)))['y']
        tp, tn, fp, fn = 0, 0, 0, 0
        for label, prediction in zip(labels, result):
            if label == 1:
                if prediction == 1:
                    tp += 1
                else:
                    fn += 1
            else:
                if prediction == 1:
                    fp += 1
                else:
                    tn += 1
        logger.info("Confusion matrix:\n\t          |___1__|___0__| (Predicted)")
        logger.info("\t(Truth) 1 | {:04d} | {:04d} |\n\t        0 | {:04d} | {:04d} |".format(tp ,fn, fp, tn))
    except FileNotFoundError:
        pass
