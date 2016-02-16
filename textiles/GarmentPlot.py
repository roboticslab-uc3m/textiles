__author__="def"

import matplotlib.pyplot as plt

class GarmentPlot:
    @staticmethod
    def plot_rgb(image, show=True):
        plt.figure()
        plt.imshow(image)
        if show:
            plt.show()

    @staticmethod
    def plot_depth(image, show=True):
        plt.figure()
        plt.imshow(image, cmap=plt.cm.RdGy)
        plt.colorbar(ticks=[0, 500, image.max()], orientation ='horizontal', shrink=0.75, pad=0.01)
        plt.axis('off')
        plt.tight_layout()
        if show:
            plt.show()

    @staticmethod
    def plot_mask(image, show=True):
        plt.figure()
        plt.imshow(image, cmap=plt.cm.gray)
        plt.axis('off')
        if show:
            plt.show()

