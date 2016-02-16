import numpy as np

class GarmentDepthMapClustering:
    @staticmethod
    def preprocess(depth_image, mask):
        background = np.inf  # Define a depth value for background pixels
        masked_depth_image = np.where(mask==255, depth_image.transpose(), background)

        # Normalize depth map
        # scaled_depth_map = normalize_1Channel_image(masked_depth_image)

        scaled_depth_map = masked_depth_image.copy()
        # Get range of values (min, max, range)
        min_value = masked_depth_image[np.unravel_index(np.where(masked_depth_image == 0, background, masked_depth_image).argmin(), masked_depth_image.shape)]
        max_value = masked_depth_image[np.unravel_index(np.where(masked_depth_image == background, 0,masked_depth_image).argmax(), masked_depth_image.shape)]
        range_value = max_value-min_value

        # Normalize using depth range
        scaled_depth_map = np.where(scaled_depth_map != background, (scaled_depth_map - min_value) * (255/range_value), 255)
        scaled_depth_map = scaled_depth_map.astype(np.uint8)
        return scaled_depth_map

    @staticmethod
    def cluster_similar_regions(preprocessed_depth_image):
        pass