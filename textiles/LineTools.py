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
    denom = np.dot(dap, db)
    num = np.dot(dap, dp )
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
    # Convert to numpy types
    a1 = np.array(a1, dtype=np.float)
    a2 = np.array(a2, dtype=np.float)
    b1 = np.array(b1, dtype=np.float)
    b2 = np.array(b2, dtype=np.float)

    intersection = [np.NaN, np.NaN]

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
    if min(a1[0], a2[0]) <= intersection[0] <= max(a1[0], a2[0]) and min(b1[0], b2[0])  <= intersection[0] <= max(b1[0], b2[0]) and \
        min(a1[1], a2[1]) <= intersection[1] <= max(a1[1], a2[1]) and min(b1[1], b2[1])  <= intersection[1] <= max(b1[1], b2[1]):
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

def seg_intersection_line(s1, s2, l1, l2):
    """
    Calculates the intersection between a segment and a line
    :param s1: first point of the segment
    :param s2: second point of the segment
    :param l1: first point of the line
    :param l2: segment point of the line
    :return: intersection point between segments a and b. If no intersection exists returns [nan, nan] as point.
    """
    # Convert to numpy types
    s1 = np.array(s1, dtype=np.float)
    s2 = np.array(s2, dtype=np.float)
    l1 = np.array(l1, dtype=np.float)
    l2 = np.array(l2, dtype=np.float)

    intersection = [np.NaN, np.NaN]

    # Find line equation
    slope_s, slope_l = None, None

    if s1[0]-s2[0] != 0:
        slope_s = np.true_divide(s1[1] - s2[1], s1[0] - s2[0])
        intercept_s = s1[1] - slope_s * s1[0]
    else:
        x_s = s1[0]

    if l1[0]-l2[0] != 0:
        slope_l = np.true_divide(l1[1] - l2[1], l1[0] - l2[0])
        intercept_l = l1[1] - slope_l * l1[0]
    else:
        x_l = l1[0]

    # Special cases: vertical lines
    if slope_s is None:
        if slope_l is None:
            if x_s == x_l:
                return [x_s, np.NaN]
            else:
                return [np.NaN, np.NaN]
        else:
            intersection = np.array([0,0])
            intersection[0] = x_s
            intersection[1] = slope_l * x_s + intercept_l
    else:
        if slope_l is None:
            intersection = np.array([0,0])
            intersection[0] = x_l
            intersection[1] = slope_s * x_l + intercept_s
        else:
            # Typical case: solve system
            with warnings.catch_warnings():
                warnings.filterwarnings('error')
                try:
                    intersection = np.linalg.solve(np.concatenate(slope_s, slope_l), np.concatenate(intercept_s, intercept_l))
                except Warning, w:
                    return [np.NaN, np.NaN]

    # Check if intersection found is within limits to ensure that belongs to segment
    if min(s1[0], s2[0]) <= intersection[0] <= max(s1[0], s2[0]) and \
                            min(s1[1], s2[1]) <= intersection[1] <= max(s1[1], s2[1]):
        return intersection
    else:
        return [np.NaN, np.NaN]

def seg_intersects_line(s1, s2, l1, l):
    """
    Check whether a line intersects a segment or not.
    This implementation takes into account vertical lines and stuff
    :param s1: first point of the segment
    :param s2: second point of the segment
    :param l1: first point of the line
    :param l: second point of the line
    :return: True if the two segments intersect, False otherwise
    """
    return any(np.isfinite(seg_intersection(s1, s2, l1, l)))

def seg_intersects_polygon(segment, polygon):
    """
    Checks if a segment intersects with a polygon
    :param segment: Segment defined with starting and ending points ((x1, y1), (x2, y2))
    :param polygon:  Polygon defined as a collection of segments [segment1, segment2]
    :return: True if they intersect, False otherwise
    """
    return bool(len(seg_intersection_polygon(segment, polygon)))

def seg_intersection_polygon(segment, polygon):
    """
    Calculates the intersection(s) point(s) of a segment and a polygon
    :param segment: Segment defined with starting and ending points ((x1, y1), (x2, y2))
    :param polygon:  Polygon defined as a collection of segments [segment1, segment2]
    :return: list of intersection points (empty if they do not intersect)
    """
    return list(filter(lambda x: any(np.isfinite(x)),
                       [seg_intersection(segment[0], segment[1], edge[0], edge[1]) for edge in polygon]))

def line_intersects_polygon(line, polygon):
    """
    Checks if a line intersects with a polygon
    :param line: Line defined with starting and ending points ((x1, y1), (x2, y2))
    :param polygon:  Polygon defined as a collection of segments [segment1, segment2]
    :return: True if they intersect, False otherwise
    """
    return bool(len(line_intersection_polygon(line, polygon)))

def line_intersection_polygon(line, polygon):
    """
    Calculates the intersection(s) point(s) of a segment and a polygon
    :param line: Line defined with starting and ending points ((x1, y1), (x2, y2))
    :param polygon:  Polygon defined as a collection of segments [segment1, segment2]
    :return: list of intersection points (empty if they do not intersect)
    """
    return list(filter(lambda x: any(np.isfinite(x)),
                       [seg_intersection(edge[0], edge[1], line[0], line[1]) for edge in polygon]))

def contour_to_segments(contour):
    """
    Converts a contour (opencv list of points) to a list of segments
    :param contour: Opencv's contour (list of points)
    :return: List of segments
    """
    return [(start.ravel().tolist(), end.ravel().tolist()) for start, end in zip(contour, contour[1:])]

def midpoint(a, b):
    """
    Computes the midpoint of a segment defined by wo end points. It works on a integer grid only.
    :param a: End oint of the segment
    :param b: End point of the segment
    :return: Midpoint
    """
    return [a[0]+(b[0]-a[0])/2, a[1]+(b[1]-a[1])/2]

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

    assert not seg_intersects(p1, p2, p3, p4)

    p1 = np.array( [2.0, 2.0] )
    p2 = np.array( [4.0, 3.0] )

    p3 = np.array( [6.0, 0.0] )
    p4 = np.array( [6.0, 3.0] )

    assert not seg_intersects(p1, p2, p3, p4)

    p1 = np.array( [1.0, 1.0] )
    p2 = np.array( [10.0, 1.0] )

    p3 = np.array( [1.0, 2.0] )
    p4 = np.array( [10.0, 2.0] )

    assert not seg_intersects(p1, p2, p3, p4)

    contour = [[[0, 0], [50, 50]],[[50,50], [50,0]],[[50, 0], [0,0]]]
    line = [[-10, 25], [24, 25]]
    line2 = [[-10, 25], [75, 25]]
    line3 = [[-10, 25], [0, 25]]

    assert not seg_intersects_polygon(line, contour)
    assert seg_intersects_polygon(line2, contour)
    assert not seg_intersects_polygon(line3, contour)

    # new testing things
    test_seg_01 = ((-2, 0), (0, 2))
    test_seg_02 = ((0, 2), (4, 2))
    test_seg_03 = ((0, 3), (6, 3))
    test_seg_04 = ((5, 1,), (6, 2))
    polypoints = [(0,0), (5, 5), (5,0)]
    polygon = [ [start, end] for start, end in zip(polypoints, polypoints[1:]+[polypoints[0]])]

    assert not seg_intersects_polygon(test_seg_01, polygon)
    assert  seg_intersects_polygon(test_seg_02, polygon)
    assert len(seg_intersection_polygon(test_seg_02, polygon)) == 1
    assert  seg_intersects_polygon(test_seg_03, polygon)
    assert len(seg_intersection_polygon(test_seg_03, polygon)) == 2
    assert  seg_intersects_polygon(test_seg_04, polygon)
    assert len(seg_intersection_polygon(test_seg_04, polygon)) == 1
