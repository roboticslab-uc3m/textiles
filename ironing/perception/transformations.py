import numpy as np

from common.perception.Utils import points_to_file


class TrajectoryTransform:
    def __init__(self):
        self.H_image_garment = None
        self.H_garment_board = None
        self.H_board_kinfu = None
        self.H_kinfu_cam = None
        self.pixel_resolution = 0.05

        # Robot transform:
        self.H_root_cam = np.array([[0.306186,  -0.458636,  0.834208, 0.087413],
                                    [-0.918559, -0.372462,  0.132372, 0.080342],
                                    [ 0.250000, -0.806799, -0.535327, 0.686337],
                                    [ 0.000000,  0.000000,  0.000000, 1.000000]])

    def load_from_files(self, H_image_garment_file, H_garment_board_file, H_board_kinfu_file, H_kinfu_cam_file):
        self.H_garment_board = np.loadtxt(H_garment_board_file)
        self.H_board_kinfu = np.loadtxt(H_board_kinfu_file)

        # Load origin point, convert it to matrix:
        self.H_image_garment = np.identity(4)

        with open(H_image_garment_file, 'r') as f:
            file_contents = f.readlines()

        self.H_image_garment[:3,3] = [ float(i) for i in file_contents[0].split(' ')]
        self.H_image_garment[0, 3] *= -1 # Axis in image are inverted
        self.H_image_garment[1, 1] = -1
        self.H_image_garment[2, 2] = -1

        # This is a little bit special (format defined by pcl, not by us)
        self.H_kinfu_cam = np.identity(4)

        with open(H_kinfu_cam_file, 'r') as f:
            file_contents = f.readlines()

        for i, line in enumerate(file_contents):
            if 'TVector' in line:
                self.H_kinfu_cam[0,3] = float(file_contents[i+1])
                self.H_kinfu_cam[1,3] = float(file_contents[i+2])
                self.H_kinfu_cam[2,3] = float(file_contents[i+3])
            if 'RMatrix' in line:
                self.H_kinfu_cam[0,:3] = [ float(n) for n in file_contents[i+1].split(' ') if n ]
                self.H_kinfu_cam[1,:3] = [ float(n) for n in file_contents[i+2].split(' ') if n ]
                self.H_kinfu_cam[2,:3] = [ float(n) for n in file_contents[i+3].split(' ') if n ]

    def debug(self, trajectory_image_px):
        # Convert list of pixels to 3d points
        trajectory_image = [np.array([[0.005 * x, 0.005 * y, 0, 1]]).transpose() for x, y in trajectory_image_px]

        # Compute inverse transformation
        H_board_image = np.dot(np.linalg.inv(self.H_garment_board), np.linalg.inv(self.H_image_garment))
        H_kinfu_image = np.dot(np.linalg.inv(self.H_board_kinfu), H_board_image)

        # Apply transformation to all points
        return [ np.dot(H_kinfu_image, point) for point in trajectory_image ]

    def __call__(self, trajectory_image_px):
        # Convert list of pixels to 3d points
        trajectory_image = [np.array([[0.005 * x, 0.005 * y, 0, 1]]).transpose() for x, y in trajectory_image_px]

        # Compute inverse transformation
        H_cam_board = np.dot(np.linalg.inv(self.H_kinfu_cam), np.linalg.inv(self.H_board_kinfu))
        H_board_image = np.dot(np.linalg.inv(self.H_garment_board), np.linalg.inv(self.H_image_garment))
        H_cam_image = np.dot(H_cam_board, H_board_image)

        # Apply transformation to all points
        return [ np.dot(H_cam_image, point) for point in trajectory_image ]

    def relative_to_robot_root(self, trajectory_image_px):
        points = self.__call__(trajectory_image_px)
        return [np.dot(self.H_root_cam, point) for point in points]


H_image_garment_file = "/home/def/Research/jresearch/2016-07-25-textiles-ironing/hoodie1/colored_mesh_1.ply-output.pcd-origin.txt"
H_garment_board_file = "/home/def/Research/jresearch/2016-07-25-textiles-ironing/hoodie1/colored_mesh_1.ply-transform2.txt"
H_board_kinfu_file = "/home/def/Research/jresearch/2016-07-25-textiles-ironing/hoodie1/colored_mesh_1.ply-transform1.txt"
H_kinfu_cam_file = "/home/def/Research/jresearch/2016-07-25-textiles-ironing/hoodie1/0.txt"

output_file = "/home/def/Research/jresearch/2016-07-25-textiles-ironing/hoodie1/traj.pcd"


def send_trajectory_over_yarp(port, trajectory):
    import yarp
    yarp.Network.initMinimum()
    p = yarp.Port()
    p.open("/traj:o")
    yarp.Network.connect("/traj:o", port)
    trajectory_bottle = yarp.Bottle()
    for point in trajectory:
        b = trajectory_bottle.addList()
        b.addDouble(point[0][0])
        b.addDouble(point[1][0])
        b.addDouble(point[2][0])
    p.write(trajectory_bottle)


if __name__ == '__main__':
    t = TrajectoryTransform()
    t.load_from_files(H_image_garment_file, H_garment_board_file, H_board_kinfu_file, H_kinfu_cam_file)

    #trajectory_image_px = [(0,0), (0, 69), (113, 0), (113,69),(48, 47), (49, 48), (49, 49), (49, 50), (49, 51), (50, 52), (51, 53), (52, 54), (53, 55), (54, 56), (55, 57), (55, 58), (56, 59), (56, 60), (56, 61), (56, 62), (56, 63), (57, 64), (58, 64), (59, 64)]
    trajectory_image_px = [(74, 18), (73, 18), (72, 17), (71, 17), (70, 17), (69, 16), (68, 15), (68, 14), (67, 13), (67, 12), (67, 11), (66, 10), (65, 9), (65, 8), (65, 7), (66, 6), (66, 5), (66, 4), (66, 3), (67, 2), (68, 2), (69, 2)]
    trajectory_cam = t(trajectory_image_px)
    print([ (float(x), float(y), float(z)) for x, y, z, w in trajectory_cam])
    trajectory_debug = t.debug(trajectory_image_px)

    points_to_file(trajectory_debug, output_file)

    trajectory_root = t.relative_to_robot_root(trajectory_image_px)
    print([ (float(x), float(y), float(z)) for x, y, z, w in trajectory_root])

    # send_trajectory_over_yarp("/read", trajectory_root)


