import numpy as np
from scipy import ndimage
from skimage import morphology
from skimage import filters
from skimage import util
from skimage import restoration
from skimage import exposure

class GarmentDepthMapClustering:
    @staticmethod
    def preprocess(depth_image, mask):
        """
        Removes the background information and normalizes the depth image range to 8-bit unsigned.
        :param depth_image:
        :param mask: Segmentation mask where white is garment and black is background
        :return: Depth image normalized and converted to 8-bit unsigned
        """
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
    def preprocess_heightmap(heightmap, mask):
        """
        Removes the background information and normalizes the depth image range to 8-bit unsigned. To be used in a heightmap (height relative to the table).
        :param heighmap:
        :param mask: Segmentation mask where white is garment and black is background
        :return: Depth image normalized and converted to 8-bit unsigned compatible with depth-image-based algorithms
        """
        background = np.inf  # Define a depth value for background pixels
        masked_depth_image = np.where(mask==255, heightmap.transpose(), background)
        scaled_depth_map = masked_depth_image.copy()

        # Normalize using height range
        max_value = masked_depth_image[np.unravel_index(np.where(masked_depth_image == background, 0,masked_depth_image).argmax(), masked_depth_image.shape)]
        scaled_depth_map = np.where(scaled_depth_map != background, (max_value - scaled_depth_map) * (255/max_value), 255)
        scaled_depth_map = scaled_depth_map.astype(np.uint8)
        return scaled_depth_map

    @staticmethod
    def cluster_similar_regions(preprocessed_depth_image, mask=None):
        """
        Apply clustering algorithm to group similar regions. This uses watershed currently.
        :param preprocessed_depth_image: Depth image normalized and converted to 8-bit unsigned
        :param mask: segmentation mask for the depth map
        :return: Labeled image after the clustering process. Each label is the median value of each cluster
        """
        scaled_depth_map = util.img_as_ubyte(preprocessed_depth_image)

        # denoise image
        denoised = restoration.denoise_tv_chambolle(scaled_depth_map, weight=0.05)
        denoised_equalize= exposure.equalize_hist(denoised)

        # find continuous region (low gradient) --> markers
        markers = filters.rank.gradient(denoised_equalize, morphology.disk(5)) < 10 # 25,15  10,10

        markers = ndimage.label(markers)[0]

        # local gradient
        gradient = filters.rank.gradient(denoised, morphology.disk(5))

        # labels
        labels = morphology.watershed(gradient, markers, mask=mask)

        # Change labels by median value of each region
        unique_labels = np.unique(labels)
        avg = np.zeros(labels.shape, np.uint8)

        for unique_value in unique_labels:
            avg_value = np.median(np.extract(labels == unique_value, preprocessed_depth_image))
            avg = np.where(labels == unique_value, int(avg_value), avg)

        return avg