import numpy as np
import warnings

__author__ = 'def'

# see Computer Graphics by F.S. Hill
# http://stackoverflow.com/questions/3252194/numpy-and-line-intersections

def perp(a):
    """
    Calculate the vector perpendicular to other vector in 2D
    :param a: Input vector (2D)
    :return: Vector perpendicular to a (2D)
    """
    b = np.empty_like(a)
    b[0] = -a[1]
    b[1] = a[0]
    return b


def line_intersection(a1, a2, b1, b2):
    """
    Calculate the intersection between two lines defined by two points a and b
    :param a1: first point of the line a
    :param a2: second point of the line a
    :param b1: first point of the line b
    :param b2: second point of the line b
    :return: intersection point between line a and b. If no intersection exists returns [nan, nan] as point.
    """
    da = a2-a1
    db = b2-b1
    dp = a1-b1
    dap = perp(da)
    denom = np.dot( dap, db)
    num = np.dot( dap, dp )
    return (num / denom.astype(float))*db + b1

def line_intersects(a1, a2, b1, b2):
    """
    Checks if two lines intersect
    :param a1: first point of the line a
    :param a2: second point of the line a
    :param b1: first point of the line b
    :param b2: second point of the line b
    :return: True if the two line intersect, False otherwise
    """
    return all(np.isfinite(line_intersection(a1, a2, b1, b2)))


def seg_intersection(a1, a2, b1, b2):
    """
    Calculates the intersection of two segments.
    This implementation takes into account vertical lines and stuff
    :param a1: first point of the segment a
    :param a2: second point of the segment a
    :param b1: first point of the segment b
    :param b2: second point of the segment b
    :return: intersection point between segments a and b. If no intersection exists returns [nan, nan] as point.
    """
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
    if slope_a is None:
        if slope_b is None:
            if x_a == x_b:
                return [x_a, np.NaN]
            else:
                return [np.NaN, np.NaN]
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
                    return [np.NaN, np.NaN]

        # Check if intersection found is within limits to ensure that belongs to segment
        if np.minimum(a1[0], a2[0]) <= intersection[0] <= np.maximum(a1[0], a2[0]) and np.minimum(b1[0], b2[0])  <= intersection[0] <= np.maximum(b1[0], b2[0]) and \
            np.minimum(a1[1], a2[1]) <= intersection[1] <= np.maximum(a1[1], a2[1]) and np.minimum(b1[1], b2[1])  <= intersection[1] <= np.maximum(b1[1], b2[1]):
            return intersection
        else:
            return [np.NaN, np.NaN]

def seg_intersects(a1, a2, b1, b2):
    """
    Check whether two segments intersect or not.
    This implementation takes into account vertical lines and stuff
    :param a1: first point of the segment a
    :param a2: second point of the segment a
    :param b1: first point of the segment b
    :param b2: second point of the segment b
    :return: True if the two segments intersect, False otherwise
    """
    return any(np.isfinite(seg_intersection(a1, a2, b1, b2)))

def seg_intersects_polygon(segment, polygon):
    """
    Checks if a segment intersects with a polygon
    :param segment: Segment defined with starting and ending points ((x1, y1), (x2, y2))
    :param polygon:  Polygon defined as a collection of segments [segment1, segment2]
    :return: True if they intersect, False otherwise
    """
    return bool(len(seg_intersection_polygon(segment, polygon)))

def seg_intersection_polygon(segment, polygon):
    intersections = [seg_intersection(np.array(segment[0], dtype=np.float), np.array(segment[1], dtype=np.float),
                              np.array(edge[0], dtype=np.float), np.array(edge[1], dtype=np.float))
                              for edge in polygon]
    return [intersection for intersection in intersections if any(np.isfinite(intersection))]


if __name__ == "__main__":
    # Testing line intersection
    p1 = np.array( [0.0, 0.0] )
    p2 = np.array( [1.0, 1.0] )

    p3 = np.array( [1.0, 3.0] )
    p4 = np.array( [2.0, 1.0] )

    print "Line intersection: ", line_intersection(p1, p2, p3, p4)

    # Testing line intersection (boolean)
    print "Line intersects?: ", line_intersects(p1, p2, p3, p4)
    assert line_intersects(p1, p2, p3, p4)

    p1 = np.array( [0.0, 0.0] )
    p2 = np.array( [1.0, 1.0] )

    p3 = np.array( [0.0, 1.0] )
    p4 = np.array( [1.0, 2.0] )

    print "Line intersects?: ", line_intersects(p1, p2, p3, p4)
    assert not line_intersects(p1, p2, p3, p4)

    # Testing segment intersection
    p1 = np.array( [0.0, 0.0] )
    p2 = np.array( [1.0, 0.0] )

    p3 = np.array( [4.0, -5.0] )
    p4 = np.array( [4.0, 2.0] )

    print seg_intersects(p1, p2, p3, p4)

    p1 = np.array( [2.0, 2.0] )
    p2 = np.array( [4.0, 3.0] )

    p3 = np.array( [6.0, 0.0] )
    p4 = np.array( [6.0, 3.0] )

    print seg_intersects(p1, p2, p3, p4)

    p1 = np.array( [1.0, 1.0] )
    p2 = np.array( [10.0, 1.0] )

    p3 = np.array( [1.0, 2.0] )
    p4 = np.array( [10.0, 2.0] )

    print seg_intersects(p1, p2, p3, p4)

    contour = [[[0, 0], [50, 50]],[[50,50], [50,0]],[[50, 0], [0,0]]]
    line = [[-10, 25], [24, 25]]
    line2 = [[-10, 25], [75, 25]]
    line3 = [[-10, 25], [0, 25]]

    print seg_intersects_polygon(line, contour)
    print seg_intersects_polygon(line2, contour)
    print seg_intersects_polygon(line3, contour)
    assert not seg_intersects_polygon(line3, contour)