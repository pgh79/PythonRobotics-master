
import numpy as np
import matplotlib.pyplot as plt
import cv2
from skimage import morphology,data,color
import random


def rgb2gray(rgb):
    return np.dot(rgb[..., :3], [0.299, 0.587, 0.114])


def grey2bin(grey):
    return np.where(grey > 0.5, 1.0, 0)


if __name__ == '__main__':
    im = np.array(plt.imread('2_3_label.png'))

    im_grey = rgb2gray(im)

    im_bin = grey2bin(im_grey)

    im_small = cv2.resize(im_bin, (250, 250))

    im_skeleton = morphology.skeletonize(im_small)
    can_go_all = np.where(im_skeleton == 1)
    cx_all = can_go_all[1]
    cy_all = can_go_all[0]
    can_go_all = np.vstack((cx_all,cy_all))
    initial_position = can_go_all[:,random.randint(1,len(cx_all))]



    plt.imshow(im_skeleton)
    plt.show()
