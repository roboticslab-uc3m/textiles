__author__ = 'def'

class ClothContour:

    def __init__(self, points):
        # Save contour points
        self.points = points
        self.n_points = len(points)

        # Generate segments:
        self.segments = self.get_contour_segments(points)

        # Generate midpoints:
        self.midpoints = self.get_contour_midpoints(self.segments)


    @staticmethod
    def get_contour_segments(contour):
        contour_segments = []

        for i, j in zip(range(-1, len(contour)-1), range(len(contour))):
            start = contour[i][0]
            end = contour[j][0]
            contour_segments.append((start, end))

        return contour_segments

    @staticmethod
    def get_contour_midpoints(segments):
        contour_midpoints = []

        for segment in segments:
            start = segment[0]
            end = segment[1]

            midpoint = [start[0]+(end[0]-start[0])/2, start[1]+(end[1]-start[1])/2]

            contour_midpoints.append(midpoint)

        return contour_midpoints

    def get_candidate_paths(self, source_points):
        candidate_paths = []
        for point in source_points:
            candidate_paths+=[ (point, target_midpoint ) for target_midpoint in self.midpoints]

        return candidate_paths