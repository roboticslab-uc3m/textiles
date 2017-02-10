# coding=utf-8

import numpy as np
from skimage import io

from textiles.ironing.perception.Li import DiscontinuityScanLi
from textiles.ironing.perception.Li import CurvatureScanLi

"""
Scan Li
-----------------------------------------------------------------------
Wrinkle detection using Discontinuity and Curvature scans, as described
in Yinxiao Li's Thesis and in article:
 * Y. Li, X. Hu, D. Xu, Y. Yue, E. Grinspun, and P. Allen, “Multi-
Sensor Surface Analysis for Robotic Ironing,” in IEEE International
Conference on Robotics and Automation (ICRA), Stockholm, 2016.

Finds the most suitable paths for ironing depending on wrinkles
(discontinuities) and height bumps.
"""


class ScanLi(object):
    def __init__(self):
        self.discontinuity_scanner = DiscontinuityScanLi.DiscontinuityScanLi()
        self.curvature_scanner = CurvatureScanLi.CurvatureScanLi()

    def load_images(self, image_folder, image_id=0, use_roi=True):
        self.discontinuity_scanner.load_images(image_folder, image_id, use_roi)
        self.discontinuity_scanner.load_svm()
        self.curvature_scanner.load_images(image_folder, image_id, use_roi)

    def run(self, debug):
        curvatures = self.curvature_scanner.run(debug)
        _, discontinuities = self.discontinuity_scanner.run(debug)

        result = np.bitwise_and(curvatures, discontinuities)

        if debug:
            io.imshow(result)
