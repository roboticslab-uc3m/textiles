import numpy as np

class TrajectoryTransform:
    def __init__(self):
        self.H_image_garment = None
        self.H_garment_board = None
        self.H_board_kinfu = None
        self.H_kinfu_cam = None
        self.pixel_resolution = 0.05

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

    def __call__(self, trajectory_image_px):
        # Convert list of pixels to 3d points
        trajectory_image = [np.array([[0.005 * x, 0.005 * y, 0, 1]]).transpose() for x, y in trajectory_image_px]

        # Compute inverse transformation
        

H_image_garment_file = "/home/def/Research/jresearch/2016-07-25-textiles-ironing/hoodie1/colored_mesh_1.ply-output.pcd-origin.txt"
H_garment_board_file = "/home/def/Research/jresearch/2016-07-25-textiles-ironing/hoodie1/colored_mesh_1.ply-transform2.txt"
H_board_kinfu_file = "/home/def/Research/jresearch/2016-07-25-textiles-ironing/hoodie1/colored_mesh_1.ply-transform1.txt"
H_kinfu_cam_file = "/home/def/Research/jresearch/2016-07-25-textiles-ironing/hoodie1/0.txt"

if __name__ == '__main__':
    t = TrajectoryTransform()
    t.load_from_files(H_image_garment_file, H_garment_board_file, H_board_kinfu_file, H_kinfu_cam_file)

    print t.H_image_garment
    print t.H_garment_board
    print t.H_board_kinfu
    print t.H_kinfu_cam

    trajectory_image_px = [(0,0), (0, 69), (113, 0), (113,69),(48, 47), (49, 48), (49, 49), (49, 50), (49, 51), (50, 52), (51, 53), (52, 54), (53, 55), (54, 56), (55, 57), (55, 58), (56, 59), (56, 60), (56, 61), (56, 62), (56, 63), (57, 64), (58, 64), (59, 64)]

    trajectory_image = [np.array([[0.005 * x, 0.005 * y, 0, 1]]).transpose() for x, y in trajectory_image_px]

    print 'image'
    print trajectory_image[0]
    print trajectory_image[1]
    print '-----'

    #H_image_garment = np.array([[0.866,0.5,0,0.290653],
    #                            [0.5,0.866,0,0.156882],
    #                            [0,0,1,0],
    #                            [0,0,0,1]])

    H_image_garment = np.array([[1,0,0,0.290653],
                                [0, -1,0,0.190781],
                                [0,0,-1,0],
                                [0,0,0,1]])

    H_garment_image = np.linalg.inv(H_image_garment)
    trajectory_garment = [ np.dot(H_garment_image, point) for point in trajectory_image ]
    print 'garment'
    print trajectory_garment[0]
    print trajectory_garment[1]
    print '-----'

    H_garment_board = np.array([[1, 0, 0, -0.871986],
                                [0, 1, 0, -0.475175],
                                [0, 0, 1, -0.0744163],
                                [0, 0, 0,  1        ]])
    H_board_garment = np.linalg.inv(H_garment_board)

    trajectory_board = [ np.dot(H_board_garment, point) for point in trajectory_garment ]
    print 'board'
    print trajectory_board[0]
    print trajectory_board[1]
    print '-----'

    R_board_kinfu = np.array([[0.814409,  0.513788, -0.269741, 0],
                              [0.513788, -0.422363,  0.746747, 0],
                              [0.269741, -0.746747, -0.607954, 0],
                              [0,         0,         0,         1]])

    T_board_kinfu = np.array([[1,  0,  0, 0.235493],
                              [0,  1,  0, -0.651935],
                              [0,  0,  1, -0.530764],
                              [0,  0,  0,         1]])

    H_board_kinfu = np.array([[0.814409,  0.513788, -0.269741,  0.235493],
                              [0.513788, -0.422363,  0.746747, -0.651935],
                              [0.269741, -0.746747, -0.607954, -0.530764],
                              [0,         0,         0,         1]])

    #H_kinfu_board = np.dot(np.linalg.inv(T_board_kinfu), np.linalg.inv(R_board_kinfu))
    H_kinfu_board = np.linalg.inv(H_board_kinfu)


    trajectory_kinfu1 = [ np.dot(np.linalg.inv(R_board_kinfu), point) for point in trajectory_board ]
    trajectory_kinfu = [ np.dot(np.linalg.inv(T_board_kinfu), point) for point in trajectory_kinfu1 ]
    print 'kinfu'
    print trajectory_kinfu[0]
    print trajectory_kinfu[1]
    print '-----'


    H_kinfu_cam = np.array([[1, 0, 0,  0.756573],
                            [0, 1, 0,  0.751346],
                            [0, 0, 1, -0.150508],
                            [0, 0, 0,    1]])
    H_cam_kinfu = np.linalg.inv(H_kinfu_cam)

    trajectory_cam = [ np.dot(H_cam_kinfu, point) for point in trajectory_kinfu ]
    print 'cam'
    print trajectory_cam[0]
    print trajectory_cam[1]


    with open("/home/def/Research/jresearch/2016-07-25-textiles-ironing/hoodie1/traj.pcd", 'w') as f:
        # Write pcd header
        f.write(
        """# PCD v.7 - Point Cloud Data file format
    VERSION .7
    FIELDS x y z
    SIZE 4 4 4
    TYPE F F F
    COUNT 1 1 1
    WIDTH {0:d}
    HEIGHT 1
    VIEWPOINT 0 0 0 1 0 0 0
    POINTS {0:d}
    DATA ascii
    """.format(len(trajectory_cam)))

        for point in trajectory_kinfu:
            f.write("{} {} {}\n".format(point[0][0], point[1][0], point[2][0]))
