import contextlib
from abc import abstractmethod

import numpy as np
import yarp


class YarpCamera(object):
    """
    Abstract base class for cameras using yarp
    """
    def __init__(self, port, resolution):
        self.port = port
        self.width, self.height = resolution

    @abstractmethod
    def get_image(self):
        """ Returns a numpy array with the image data """


class YarpRGBCamera(YarpCamera):
    """
    RGB camera using yarp
    """
    def __init__(self, port, resolution):
        super(YarpRGBCamera, self).__init__(port, resolution)

    def get_image(self):
        # Create image placeholder
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        # Read image with yarp
        yarp_img = yarp.ImageRgb()
        yarp_img.resize(self.width, self.height)
        yarp_img.setExternal(img, self.width, self.height)
        if not self.port.read(yarp_img):
            raise ValueError("Try to read a closed port")

        # Check image
        if not yarp_img.getRawImage().__long__() == img.__array_interface__['data'][0]:
            raise RuntimeError("read() reallocated my yarp_img")

        return img


class YarpDepthCamera(YarpCamera):
    # def __init__(self, port, resolution):
    #     super(YarpDepthCamera, self).__init(port, resolution)

    def get_image(self):
        # Create image placeholder
        img = np.zeros((self.height, self.width), dtype=np.uint16)

        # Read image with yarp
        yarp_img = yarp.ImageMono16()
        yarp_img.resize(self.width, self.height)
        yarp_img.setExternal(img, self.width, self.height)
        if not self.port.read(yarp_img):
            raise ValueError("Try to read a closed port")

        # Check image
        if not yarp_img.getRawImage().__long__() == img.__array_interface__['data'][0]:
            raise RuntimeError("read() reallocated my yarp_img")

        return img


class YarpNetworkException(RuntimeError):
    """ Exception raised when some error ocurred in the yarp network (exception body explains the error) """


@contextlib.contextmanager
def get_cameras(rgb_camera, rgb_resolution, depth_camera, depth_resolution):
    # Start yarp network
    yarp.Network.init()
    if not yarp.Network.checkNetwork():
        raise YarpNetworkException("Yarp network could not be found")

    # Connect to rgb camera
    rgb_camera_port = yarp.Port()
    rgb_camera_port.open("/python/rgb:i")
    if not yarp.Network.connect(rgb_camera, "/python/rgb:i"):
        raise YarpNetworkException("Could not connect to RGB camera stream")
    rgb = YarpRGBCamera(rgb_camera_port, rgb_resolution)

    # Connect to depth camera
    depth_camera_port = yarp.Port()
    depth_camera_port.open("/python/depth:i")
    if not yarp.Network.connect(depth_camera, "/python/depth:i"):
        raise YarpNetworkException("Could not connect to depth camera stream")
    depth = YarpDepthCamera(depth_camera_port, depth_resolution)

    # Give cameras through context manager
    yield rgb, depth

    # Cleanup
    rgb_camera_port.close()
    depth_camera_port.close()

    yarp.Network.fini()

if __name__ == '__main__':
    # Small example of how to use this file
    import matplotlib.pyplot as plt

    with get_cameras("/OpenNI2/imageFrame:o", (640, 480),
                     "/OpenNI2/depthFrame:o", (640, 480)) as (rgb_camera, depth_camera):
        rgb_image = rgb_camera.get_image()
        depth_image = depth_camera.get_image()

        plt.figure()
        plt.title("RGB image")
        plt.imshow(rgb_image)

        plt.figure()
        plt.title("Depth image")
        plt.imshow(depth_image, cmap=plt.cm.RdGy)

        plt.show()
