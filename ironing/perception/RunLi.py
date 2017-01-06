#!/usr/env python3
# coding=utf-8
import begin
import logging

import DiscontinuityScanLi

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
def predict(svm_datafile: 'File storing the SVM parameters' = '', image_id: 'Id of the image to use for prediction' = 0,
        display_results: 'Show feedback of the results' = False, *image_folder):
    DiscontinuityScanLi.predict(svm_datafile, image_id, display_results, image_folder)

@begin.start(auto_convert=True)
@begin.logging
def main():
    pass