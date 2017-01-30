import cv2

def points_to_file(points, output_file):
    """
    Save a series of 3D points to a PCL's PCD file
    :param points: a list containing the points (each point is either a 3-tuple or a numpy array)
    :param output_file: file to save the points in
    """
    with open(output_file, 'w') as f:
        # Write pcd header
        f.write("""# PCD v.7 - Point Cloud Data file format
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
""".format(len(points)))

        for point in points:
            try:
                # Try this for numpy points
                f.write("{} {} {}\n".format(point[0][0], point[1][0], point[2][0]))
            except TypeError:
                # Otherwise point is a tuple
                f.write("{} {} {}\n".format(point[0], point[1], point[2]))


def sparse2dense(mask):
    """
    From a sparse segmentation mask (obtained from a point cloud) get a dense mask
    """
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Fill large holes
    mask_outlines, dummy = cv2.findContours(closing.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.fillPoly(closing, mask_outlines, 255)

    return closing
