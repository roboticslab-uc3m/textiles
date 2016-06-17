import numpy as np
import matplotlib.pyplot as plt

def figure_7_2():
    input_filename = '../data/20150625_2/sweater01_1_fold.mat'
    img = np.loadtxt(input_filename)
    img = img.transpose()
    img = np.fliplr(img)

    fig = plt.imshow(img, cmap=plt.cm.RdGy)
    plt.colorbar(ticks=[0, 500, img.max()], orientation ='horizontal', shrink=0.75, pad=0.01)
    plt.axis('off')
    plt.tight_layout()
    plt.show()

figure_7_2()

