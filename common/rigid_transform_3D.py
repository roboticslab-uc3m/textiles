# Code downloaded from: http://nghiaho.com/?page_id=671
# Author: Nghia Ho

import numpy as np
#from math import sqrt

# Input: expects Nx3 matrix of points
# Returns R,t
# R = 3x3 rotation matrix
# t = 3x1 column vector

def rigid_transform_3D(A, B):
    assert len(A) == len(B)

    N = A.shape[0]; # total points

    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    
    # centre the points
    AA = A - np.tile(centroid_A, (N, 1))
    BB = B - np.tile(centroid_B, (N, 1))

    # dot is matrix multiplication for array
    H = np.dot(AA.transpose(), BB)

    U, S, Vt = np.linalg.svd(H)

    R = np.dot(Vt.T, U.T)

    # special reflection case
    if np.linalg.det(R) < 0:
       print("Reflection detected")
       Vt[2,:] *= -1
       R = np.dot(Vt.T, U.T)

    t = np.dot(-R, centroid_A.T) + centroid_B.T

    print(t)

    return R, t

if __name__ == '__main__':
    # Test with random data

    # Random rotation and translation
    R = np.mat(np.random.rand(3,3))
    t = np.mat(np.random.rand(3,1))

    # make R a proper rotation matrix, force orthonormal
    U, S, Vt = np.linalg.svd(R)
    R = np.dot(U,Vt)

    # remove reflection
    if np.linalg.det(R) < 0:
       Vt[2,:] *= -1
       R = np.dot(U,Vt)

    # number of points
    n = 10

    A = np.mat(np.random.rand(n,3));
    B = np.dot(R,A.T) + np.tile(t, (1, n))
    B = B.T;

    # recover the transformation
    ret_R, ret_t = rigid_transform_3D(A, B)

    A2 = (np.dot(ret_R,A.T)) + np.tile(ret_t, (1, n))
    A2 = A2.T

    # Find the error
    err = A2 - B

    err = np.multiply(err, err)
    err = sum(err)
    rmse = np.sqrt(err/n);

    print("Points A")
    print(A)
    print("")

    print("Points B")
    print(B)
    print("")

    print("Rotation")
    print(R)
    print("")

    print("Translation")
    print(t)
    print("")

    print("Computed:")
    print(ret_R, ret_t)

    print("RMSE:", rmse)
    print("If RMSE is near zero, the function is correct!")
