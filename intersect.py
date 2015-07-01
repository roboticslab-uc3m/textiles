import numpy as np
import warnings

__author__ = 'def'

# see Computer Graphics by F.S. Hill
# http://stackoverflow.com/questions/3252194/numpy-and-line-intersections

def perp( a ) :
    b = np.empty_like(a)
    b[0] = -a[1]
    b[1] = a[0]
    return b

# line segment a given by endpoints a1, a2
# line segment b given by endpoints b1, b2
# return
def seg_intersect(a1,a2, b1,b2) :
    with warnings.catch_warnings():
        warnings.filterwarnings('error')
        try:
            da = a2-a1
            db = b2-b1
            dp = a1-b1
            dap = perp(da)
            denom = np.dot( dap, db)
            num = np.dot( dap, dp )
            intersection = (num / denom.astype(float))*db + b1
            # print 'Intersection: ' + str(intersection)

            # Check if intersection found is within limits to ensure that belongs to segment
            # print 'X1: ' + str(np.minimum(a1[0], a2[0]))+ '<=' + str(intersection[0]) +'<='+ str(np.maximum(a1[0], a2[0]))
            # print 'X2: ' + str(np.minimum(b1[0], b2[0]))+ '<=' + str(intersection[0]) +'<='+ str(np.maximum(b1[0], b2[0]))
            # print 'Y1: ' + str(np.minimum(a1[1], a2[1]))+ '<=' + str(intersection[1]) +'<='+ str(np.maximum(a1[1], a2[1]))
            # print 'Y2: ' + str(np.minimum(b1[1], b2[1]))+ '<=' + str(intersection[1]) +'<='+ str(np.maximum(b1[1], b2[1]))
            if np.minimum(a1[0], a2[0]) <= intersection[0] <= np.maximum(a1[0], a2[0]) and np.minimum(b1[0], b2[0])  <= intersection[0] <= np.maximum(b1[0], b2[0]) and \
               np.minimum(a1[1], a2[1]) <= intersection[1] <= np.maximum(a1[1], a2[1]) and np.minimum(b1[1], b2[1])  <= intersection[1] <= np.maximum(b1[1], b2[1]):
                return True
            else:
                return False

        except Warning, w:
            return False

def my_seg_intersect(a1, a2, b1, b2):
    intersection = None

    # Find line equation
    slope_a, slope_b = None, None

    if a1[0]-a2[0] != 0:
        slope_a = np.true_divide(a1[1]-a2[1], a1[0]-a2[0])
        intercept_a = a1[1] - slope_a * a1[0]
    else:
        x_a = a1[0]

    if b1[0]-b2[0] != 0:
        slope_b = np.true_divide(b1[1]-b2[1], b1[0]-b2[0])
        intercept_b = b1[1] - slope_b * b1[0]
    else:
        x_b = b1[0]

    # Special cases: vertical lines
    # print "Slopes: " + str(slope_a) + ' ' + str(slope_b)
    if slope_a is None:
        if slope_b is None:
            if x_a == x_b:
                return True
            else:
                return False
        else:
            intersection = np.array([0,0])
            intersection[0] = x_a
            intersection[1] = slope_b * x_a + intercept_b
    else:
        if slope_b is None:
            intersection = np.array([0,0])
            intersection[0] = x_b
            intersection[1] = slope_a * x_b + intercept_a

        else:
            # Typical case: solve system
            with warnings.catch_warnings():
                warnings.filterwarnings('error')
                try:
                    intersection = np.linalg.solve(np.concatenate(slope_a, slope_b), np.concatenate(intercept_a, intercept_b))
                except Warning, w:
                    return False

        # print "Intersection: " + str(intersection)

        # Check if intersection found is within limits to ensure that belongs to segment
        # print 'X1: ' + str(np.minimum(a1[0], a2[0]))+ '<=' + str(intersection[0]) +'<='+ str(np.maximum(a1[0], a2[0]))
        # print 'X2: ' + str(np.minimum(b1[0], b2[0]))+ '<=' + str(intersection[0]) +'<='+ str(np.maximum(b1[0], b2[0]))
        # print 'Y1: ' + str(np.minimum(a1[1], a2[1]))+ '<=' + str(intersection[1]) +'<='+ str(np.maximum(a1[1], a2[1]))
        # print 'Y2: ' + str(np.minimum(b1[1], b2[1]))+ '<=' + str(intersection[1]) +'<='+ str(np.maximum(b1[1], b2[1]))
        if np.minimum(a1[0], a2[0]) <= intersection[0] <= np.maximum(a1[0], a2[0]) and np.minimum(b1[0], b2[0])  <= intersection[0] <= np.maximum(b1[0], b2[0]) and \
            np.minimum(a1[1], a2[1]) <= intersection[1] <= np.maximum(a1[1], a2[1]) and np.minimum(b1[1], b2[1])  <= intersection[1] <= np.maximum(b1[1], b2[1]):
            return True
        else:
            return False



def polygon_intersect(segment, polygon):
    for edge in polygon:
        if seg_intersect(np.array(segment[0], dtype=np.float), np.array(segment[1], dtype=np.float), np.array(edge[0], dtype=np.float), np.array(edge[1], dtype=np.float)):
            return True
    return False




if __name__ == "__main__":
    p1 = np.array( [0.0, 0.0] )
    p2 = np.array( [1.0, 0.0] )

    p3 = np.array( [4.0, -5.0] )
    p4 = np.array( [4.0, 2.0] )

    print my_seg_intersect( p1,p2, p3,p4)

    p1 = np.array( [2.0, 2.0] )
    p2 = np.array( [4.0, 3.0] )

    p3 = np.array( [6.0, 0.0] )
    p4 = np.array( [6.0, 3.0] )

    print my_seg_intersect( p1,p2, p3,p4)

    p1 = np.array( [1.0, 1.0] )
    p2 = np.array( [10.0, 1.0] )

    p3 = np.array( [1.0, 2.0] )
    p4 = np.array( [10.0, 2.0] )

    print my_seg_intersect( p1,p2, p3,p4)

    contour = [[[0, 0], [50, 50]],[[50,50], [50,0]],[[50, 0], [0,0]]]
    line = [[-10, 25], [24, 25]]
    line2 = [[-10, 25], [75, 25]]

    print polygon_intersect(line, contour)
    print polygon_intersect(line2, contour)