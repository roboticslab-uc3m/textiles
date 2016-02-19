__author__ = 'def'

from LineTools import seg_intersects, seg_intersects_polygon, contour_to_segments

class ClothContour:

    def __init__(self, points):
        # Save contour points
        self.points = points
        self.n_points = len(points)

        # Generate segments:
        self.segments = contour_to_segments(points)
        # print self.segments

        # Generate midpoints:
        self.midpoints = self.get_contour_midpoints(self.segments)

    @staticmethod
    def get_contour_midpoints(segments):
        contour_midpoints = [[start[0]+(end[0]-start[0])/2, start[1]+(end[1]-start[1])/2] for start, end in segments]
        return contour_midpoints

    def get_candidate_paths(self, source_points):
        candidate_paths = []
        for point in source_points:
            candidate_paths+=[ (i, (point, target_midpoint )) for i, target_midpoint in enumerate(self.midpoints)]

        return candidate_paths

    def get_valid_paths(self, source_points):
        candidate_paths = self.get_candidate_paths(source_points)
        valid_paths = []
        for id, path in candidate_paths:
            polygon = [segment for i, segment in enumerate(self.segments) if i != id]
            if not seg_intersects_polygon(path, polygon):
                valid_paths.append((id, path))
            else:
                valid_paths.append((id, None))

        return valid_paths