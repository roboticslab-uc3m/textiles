#!/usr/env python3
# coding=utf-8

import DiscontinuityScanLi
import ScanLi

import begin
import os

"""
Run Li
----------------------------------------------------------------------------
Runs the different scans implemented in CurvatureScanLi, DiscontinuityScanLi
and ScanLi.
"""


@begin.subcommand
@begin.convert(_automatic=True)
def generate_dataset(num_images: 'Number of images in image folder' = 0, display_results: 'Show feedback of the process' = False,
          *image_folder):
    DiscontinuityScanLi.generate_dataset(num_images, display_results, image_folder)


@begin.subcommand
@begin.convert(_automatic=True)
def compute_sift(num_images: 'Number of images in image folder' = 0, display_results: 'Show feedback of the process' = False,
          *image_folder):
    """
    Computes the SIFT descriptors of the labeled images
    """
    DiscontinuityScanLi.compute_SIFT(num_images, display_results, image_folder)


@begin.subcommand
@begin.convert(_automatic=True)
def train_svm(num_images: 'Number of images in image folder' = 0, *image_folder):
    """
    Loads SIFT data from the specified path and trains a SVM with them
    """
    DiscontinuityScanLi.train_svm(num_images, image_folder)


@begin.subcommand
@begin.convert(_automatic=True)
def predict(image_id: 'Id of the image to use for prediction' = 0, display_results: 'Show feedback of the results' = False,
            *image_folder):
    DiscontinuityScanLi.predict(image_id, display_results, image_folder)


@begin.subcommand
@begin.convert(_automatic=True)
def run(image_id: 'Id of the image to use for analysis' = 0, display_results: 'Show feedback of the results' = False,
            *image_folder):
    image_folder = map(lambda x: os.path.abspath(os.path.expanduser(x)), image_folder)
    image_folder = list(image_folder)[0]

    scanner = ScanLi.ScanLi()
    scanner.load_images(image_folder, image_id)
    scanner.run(display_results)


@begin.start(auto_convert=True)
@begin.logging
def main():
    pass