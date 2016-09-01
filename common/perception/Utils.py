def points_to_file(points, output_file):
    with open(output_file, 'w') as f:
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
    """.format(len(points)))

        for point in points:
            try:
                # Try this for numpy points
                f.write("{} {} {}\n".format(point[0][0], point[1][0], point[2][0]))
            except TypeError:
                # Otherwise point is a tuple
                f.write("{} {} {}\n".format(point[0], point[1], point[2]))