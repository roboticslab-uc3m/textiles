"""
Transformer.py
---------------------
Transform coordinate from pixel coordinate to world coordinates in the camera frame of reference
"""

from ironing.perception.transformations import TrajectoryTransform

class Transformer:
    """
    Transforms coordinates from pixel coordinates to world coordinates in the camera frame of reference
    """

    def add_image_params(self, center, pixel_resolution):
        """
        Add image parameters to transformer
        :param center: Pixel coordinates of the center of the image
        :param pixel_resolution: Corresponding length of each pixel in mm in the world
        """
        pass

    def add_kinfu_to_image_transform(self, T):
        """
        Add (direct) transform from kinfu to image.
        This will be inverted inside the class to revert the transformation.
        :param T: homogeneous transformation matrix from camera to image
        """
        pass


    def add_kinfu_params(self, params):
        """
        Add (direct) transform from camera to kinfu coordinates.
        This will be inverted inside the class to revert the transformation.
        :param params: homogeneous transformation matrix from camera to  kinfu
        """
        pass

    def __call__(self, points):
        """
        Apply inverse transformation to transform points in pixel coordinates to camera coordinates.
        :param points: Can be either a single point (x, y) or an iterable containing points (x, y).
        :return: Depending on input, either a single transformed point (x, y, z) or a list of (x, y, z) points.
        """
        pass

    def debug(self, points):
        """
        Apply inverse transformation to transform points in pixel coordinates to kinfu coordinates. This allows to
        plot these points in the kinfu point cloud to check that the points are being transformed correctly.
        :param points: Can be either a single point (x, y) or an iterable containing points (x, y).
        :return: Depending on input, either a single transformed point (x, y, z) or a list of (x, y, z) points.
        """
        pass

