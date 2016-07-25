import numpy as np

trajectory_image_px = [(48, 47), (49, 48), (49, 49), (49, 50), (49, 51), (50, 52), (51, 53), (52, 54), (53, 55), (54, 56), (55, 57), (55, 58), (56, 59), (56, 60), (56, 61), (56, 62), (56, 63), (57, 64), (58, 64), (59, 64)]

trajectory_image = [np.array([[0.005 * x, 0.005 * y, 0, 1]]).transpose() for x, y in trajectory_image_px]

print 'image'
print trajectory_image[0]
print trajectory_image[1]
print '-----'

H_image_garment = np.array([[1,0,0,0.290653],
                            [0,-1,0,0.156882],
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

#H_board_kinfu = np.array([[0.814409,  0.513788, -0.269741, 0.235493],
#                          [0.513788, -0.422363,  0.746747, -0.651935],
#                          [0.269741, -0.746747, -0.607954, -0.530764],
#                          [0,         0,         0,         1]])
#H_kinfu_board = np.linalg.inv(H_board_kinfu)

H_board_kinfu = np.array([[0.814409,  0.513788, -0.269741, -0.235493],
                          [0.513788, -0.422363,  0.746747, 0.651935],
                          [0.269741, -0.746747, -0.607954, 0.530764],
                          [0,         0,         0,         1]])
H_kinfu_board = H_board_kinfu


trajectory_kinfu = [ np.dot(H_kinfu_board, point) for point in trajectory_board ]
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
print trajectory_cam[-1]

origin_kinfu = np.array([[0,0,0,1]]).transpose()
print "---"
print np.dot(H_kinfu_board, origin_kinfu)