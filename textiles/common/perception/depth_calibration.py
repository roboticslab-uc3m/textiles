import numpy as np

from textiles.common.rigid_transform_3D import rigid_transform_3D

kinfu_wrt_cam = np.identity(4)
kinfu_wrt_cam[0:3, 3] = [-0.75, -0.75, +0.15]


points_kinfu = [(0.833500, 0.713380, 0.512660), # Green
                (0.777830, 1.073700, 0.556870), # Blue
                (1.106000, 1.205600, 0.460990), # Red
                (1.185100, 0.795410, 0.589020)] # Square

points_root =  [(0.8411, -0.2119, 0.3206), # Green
                (1.2249, -0.2196, 0.2604), # Blue
                (1.3314,  0.1651, 0.3661), # Red
                (0.8817,  0.2193, 0.2445)] # Square

# Points to numpy
np_points_kinfu = [np.array([[x, y, z, 1]]).transpose() for x, y, z in points_kinfu]
np_points_root = [np.array([[x, y, z, 1]]).transpose() for x, y, z in points_root]


points_cam = np.dot(kinfu_wrt_cam, np.hstack(np_points_kinfu))
print(points_cam)

R, t = rigid_transform_3D(points_cam[:3,:].transpose(), np.hstack(np_points_root)[:3,:].transpose())
print(R, t)

H_root_cam = np.identity(4)
H_root_cam[:3, :3] = R
H_root_cam[:3, 3] = t
print(H_root_cam)


