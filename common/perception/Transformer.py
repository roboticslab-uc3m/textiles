"""
Transformer.py
---------------------
Transform coordinate from pixel coordinate to world coordinates in the camera frame of reference
"""

import numpy as np
import collections

class Transformer:
    """
    Transforms coordinates from pixel coordinates to world coordinates in the camera frame of reference
    """
    def __init__(self):
        self.H_image_object = None
        self.H_object_kinfu =None
        self.H_kinfu_cam = None
        self.pixel_resolution = None
        self.H_cam_image = None
        self.H_kinfu_image = None

    def _compute_inverse_transform(self):
        """
        Using all direct transforms stored, compute the inverse transformation
        """
        self.H_kinfu_image = np.dot(np.linalg.inv(self.H_object_kinfu), np.linalg.inv(self.H_image_object))
        self.H_cam_image = np.dot(np.linalg.inv(self.H_kinfu_cam), self.H_kinfu_image)

    def add_image_params(self, image_origin, pixel_resolution):
        """
        Add image parameters to transformer
        :param image_origin: Coordinates of the image origin in local object frame
        coordinates (mm)
        :param pixel_resolution: Corresponding length of each pixel in m in the world
        """
        self.pixel_resolution = pixel_resolution

        self.H_image_object = np.identity(4)
        self.H_image_object[:2,3] = [ float(i) for i in image_origin ]
        self.H_image_object[0, 3] *= -1 # Axes in image are inverted
        self.H_image_object[1, 1] = -1
        self.H_image_object[2, 2] = -1

        try:
            self._compute_inverse_transform()
        except np.linalg.linalg.LinAlgError:
            pass


    def add_kinfu_wrt_object_transform(self, T):
        """
        Add (direct) kinfu with respect to object frame transform.
        This will be inverted inside the class to revert the transformation.
        :param T: homogeneous matrix representing the kinfu with respect to object
        frame transform
        """
        self.H_object_kinfu = T

        try:
            self._compute_inverse_transform()
        except np.linalg.linalg.LinAlgError:
            pass

    def add_kinfu_params(self, params):
        """
        Add (direct) camera with respect to kinfu coordinates transform.
        This will be inverted inside the class to revert the transformation.
        :param params: homogeneous matrix representing the camera with respect
        to kinfu transform
        """
        self.H_kinfu_cam = params

        try:
            self._compute_inverse_transform()
        except np.linalg.linalg.LinAlgError:
            pass


    def __call__(self, target):
        """
        Apply inverse transformation to transform points in pixel coordinates to
        camera coordinates.
        :param target: Can be either a single point (x, y) or an iterable
        containing points (x, y).
        :return: Depending on input, either a single transformed point (x, y, z)
        or a list of (x, y, z) points.
        """
        if isinstance(target, collections.Iterable):
            points = target
        else:
            points = [target]

        # Points to numpy
        np_points = [np.array([[self.pixel_resolution * x,
                                self.pixel_resolution * y,
                                0, 1]]).transpose() for x, y in points]

        # Apply transformation to all points
        transformed_np_points =  [ np.dot(self.H_cam_image, point) for point in np_points ]
        transformed_points = [ (float(x), float(y), float(z)) for x, y, z, w in transformed_np_points]

        if isinstance(target, collections.Iterable):
            return transformed_points
        else:
            return transformed_points[0]

    def debug(self, target):
        """
        Apply inverse transformation to transform points in pixel coordinates to
        kinfu coordinates. This allows to plot these points in the kinfu point cloud
        to check that the points are being transformed correctly.
        :param points: Can be either a single point (x, y) or an iterable containing
        points (x, y).
        :return: Depending on input, either a single transformed point (x, y, z) or
        a list of (x, y, z) points.
        """
        if isinstance(target, collections.Iterable):
            points = target
        else:
            points = [target]

        # Points to numpy
        np_points = [np.array([[self.pixel_resolution * x,
                                self.pixel_resolution * y,
                                0, 1]]).transpose() for x, y in points]

        # Apply transformation to all points
        transformed_np_points =  [ np.dot(self.H_kinfu_image, point) for point in np_points ]
        transformed_points = [ (float(x), float(y), float(z)) for x, y, z, w in transformed_np_points]

        if isinstance(target, collections.Iterable):
            return transformed_points
        else:
            return transformed_points[0]

    @staticmethod
    def load_kinfu_params_from_file(filepath):
        """
        Parse a file with the kinfu parameters
        """
        # This is a little bit special (format defined by pcl, not by us)
        H_kinfu_cam = np.identity(4)

        with open(filepath, 'r') as f:
            file_contents = f.readlines()

        for i, line in enumerate(file_contents):
            if 'TVector' in line:
                H_kinfu_cam[0,3] = float(file_contents[i+1])
                H_kinfu_cam[1,3] = float(file_contents[i+2])
                H_kinfu_cam[2,3] = float(file_contents[i+3])
            if 'RMatrix' in line:
                H_kinfu_cam[0,:3] = [ float(n) for n in file_contents[i+1].split(' ') if n ]
                H_kinfu_cam[1,:3] = [ float(n) for n in file_contents[i+2].split(' ') if n ]
                H_kinfu_cam[2,:3] = [ float(n) for n in file_contents[i+3].split(' ') if n ]



if __name__ == "__main__":
    # Just a little bit of testing to check this class is ok
    #
    def isclose(a, b, rel_tol=1e-04, abs_tol=0.0):
        """
        Test for closeness in floats
        From http://stackoverflow.com/questions/5595425/what-is-the-best-way-to-compare-floats-for-almost-equality-in-python
        """
        return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)


    # Test data
    trajectory_image_px = [(74, 18), (73, 18), (72, 17), (71, 17), (70, 17), (69, 16), (68, 15), (68, 14), (67, 13), (67, 12), (67, 11), (66, 10), (65, 9), (65, 8), (65, 7), (66, 6), (66, 5), (66, 4), (66, 3), (67, 2), (68, 2), (69, 2)]
    expected_trajectory = [(-0.10022514941662825, 0.09649845094857587, 0.7243258642402934), (-0.0955567899499925, 0.0982412354804805, 0.7239146237407784), (-0.09234443437509249, 0.10301004910777437, 0.7197989019390814), (-0.08767607490845669, 0.104752833639679, 0.7193876614395662), (-0.083007715441821, 0.1064956181715836, 0.7189764209400512), (-0.07979535986692099, 0.11126443179887743, 0.7148606991383541), (-0.07658300429202092, 0.1160332454261713, 0.710744977336657), (-0.07803900818375664, 0.11905927452156054, 0.7070404960344749), (-0.07482665260885668, 0.12382808814885438, 0.7029247742327778), (-0.0762826565005924, 0.1268541172442436, 0.6992202929305957), (-0.07773866039232813, 0.12988014633963285, 0.6955158116284136), (-0.07452630481742811, 0.13464895996692672, 0.6914000898267164), (-0.07131394924252804, 0.13941777359422058, 0.6872843680250194), (-0.07276995313426382, 0.14244380268960982, 0.6835798867228372), (-0.07422595702599954, 0.14546983178499906, 0.6798754054206552), (-0.08035032038437107, 0.1467530763484837, 0.676582164617988), (-0.08180632427610679, 0.14977910544387293, 0.672877683315806), (-0.08326232816784251, 0.15280513453926217, 0.6691732020136238), (-0.08471833205957824, 0.1558311636346514, 0.6654687207114418), (-0.09084269541794976, 0.15711440819813602, 0.6621754799087748), (-0.0955110548845855, 0.15537162366623142, 0.6625867204082898), (-0.1001794143512213, 0.1536288391343268, 0.6629979609078048)]
    expected_debug =      [(0.6524057298204631, 0.8474544193042889, 0.5733132433025772), (0.6570711330037958, 0.8492105413233111, 0.5729258846539766), (0.6602907103020192, 0.8539890752139689, 0.56882710990125), (0.6649561134853521, 0.8557451972329911, 0.5684397512526496), (0.6696215166686849, 0.8575013192520133, 0.568052392604049), (0.6728410939669083, 0.8622798531426712, 0.5639536178513224), (0.6760606712651318, 0.8670583870333292, 0.5598548430985957), (0.6746148453800226, 0.8700807989049649, 0.5561434269944695), (0.677834422678246, 0.8748593327956228, 0.5520446522417428), (0.6763885967931368, 0.8778817446672585, 0.5483332361376165), (0.6749427709080276, 0.8809041565388943, 0.5446218200334905), (0.6781623482062511, 0.8856826904295523, 0.5405230452807637), (0.6813819255044746, 0.8904612243202101, 0.5364242705280371), (0.6799360996193653, 0.8934836361918459, 0.5327128544239108), (0.6784902737342561, 0.8965060480634817, 0.5290014383197846), (0.672379044665814, 0.8977723379160952, 0.525677380864259), (0.6709332187807047, 0.900794749787731, 0.5219659647601328), (0.6694873928955954, 0.9038171616593668, 0.5182545486560066), (0.6680415670104862, 0.9068395735310024, 0.5145431325518804), (0.661930337942044, 0.9081058633836161, 0.5112190750963547), (0.6572649347587114, 0.9063497413645939, 0.5116064337449552), (0.6525995315753785, 0.9045936193455717, 0.5119937923935557)]
    image_params = ((-0.29311, 0.140912), 0.005)
    kinfu_wrt_object = np.array([[ -0.93308, -0.35122,  0.07747,  0.93887],
                                [  -0.28917,  0.60448, -0.74228,  0.15285],
                                [   0.21388, -0.71501, -0.66559,  0.84800],
                                [ 0.00000,  0.00000,  0.00000, 1.00000]])
    kinfu_params =    np.array([[ 0.99998, -0.00285, -0.00507, 0.75657],
                                [ 0.00285,  1.00000, -0.00014, 0.75135],
                                [ 0.00507,  0.00013,  0.99999,-0.15051],
                                [ 0.00000,  0.00000,  0.00000, 1.00000]])

    # Testing
    transformer = Transformer()
    transformer.add_image_params(*image_params)
    transformer.add_kinfu_wrt_object_transform(kinfu_wrt_object)
    transformer.add_kinfu_params(kinfu_params)

    # Testing transform image -> cam
    result = transformer(trajectory_image_px)

    bad_points = 0
    for (exp_x, exp_y, exp_z), (res_x, res_y, res_z) in zip(expected_trajectory, result):
         try:
            assert(isclose(exp_x, res_x))
            assert(isclose(exp_y, res_y))
            assert(isclose(exp_z, res_z))
         except AssertionError:
            print (exp_x, exp_y, exp_z), (res_x, res_y, res_z)
            bad_points += 1
            #break

    print "Results image -> cam: {}/{} bad points".format(bad_points, len(result))

    # Testing transform image -> kinfu
    result2 = transformer.debug(trajectory_image_px)

    bad_points2 = 0
    for (exp_x, exp_y, exp_z), (res_x, res_y, res_z) in zip(expected_debug, result2):
         try:
            assert(isclose(exp_x, res_x))
            assert(isclose(exp_y, res_y))
            assert(isclose(exp_z, res_z))
         except AssertionError:
            print (exp_x, exp_y, exp_z), (res_x, res_y, res_z)
            bad_points2 += 1
            #break

    print "Results image -> kinfu: {}/{} bad points".format(bad_points2, len(result2))