__author__ = 'def'

from LineTools import seg_intersects, seg_intersects_polygon

class ClothContour:

    def __init__(self, points):
        # Save contour points
        self.points = points
        self.n_points = len(points)

        # Generate segments:
        self.segments = self.get_contour_segments(points)
        # print self.segments

        # Generate midpoints:
        self.midpoints = self.get_contour_midpoints(self.segments)


    @staticmethod
    def get_contour_segments(contour):
        contour_segments = []

        for i, j in zip(range(-1, len(contour)-1), range(len(contour))):
            start = contour[i][0]
            end = contour[j][0]
            segment = (list(start), list(end))
            contour_segments.append(segment)

        return contour_segments

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
        valid_paths = [ path if not seg_intersects_polygon(path[1], [segment for i, segment in enumerate(self.segments) if i != path[0]])
                        else (path[0], None) for path in candidate_paths ]

        # for original, filtered in zip(candidate_paths, valid_paths):
        #     print original
        #     print filtered

        return valid_paths