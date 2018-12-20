import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import cv2


# 将 RGB 转为灰度图
def rgb2gray(rgb):
    return np.dot(rgb[..., :3], [0.299, 0.587, 0.114])


def grey2bin(grey):
    return np.where(grey > 0.5, 1.0, 0)


if __name__ == '__main__':
    # 将图片组合到一起生成一张整体的地图
    # im2_2 = np.array(plt.imread('2_2_label.png'))
    # im_grey2_2 = rgb2gray(im2_2)
    # im_bin2_2 = grey2bin(im_grey2_2)
    # im2_3 = np.array(plt.imread('2_3_label.png'))
    # im_grey2_3 = rgb2gray(im2_3)
    # im_bin2_3 = grey2bin(im_grey2_3)
    # im2_4 = np.array(plt.imread('2_4_label.png'))
    # im_grey2_4 = rgb2gray(im2_4)
    # im_bin2_4 = grey2bin(im_grey2_4)
    # im2_5 = np.array(plt.imread('2_5_label.png'))
    # im_grey2_5 = rgb2gray(im2_5)
    # im_bin2_5 = grey2bin(im_grey2_5)
    # image2 = np.concatenate((im_grey2_2,im_grey2_3, im_grey2_4,im_grey2_5),axis=1)
    #
    # im3_2 = np.array(plt.imread('3_2_label.png'))
    # im_grey3_2 = rgb2gray(im3_2)
    # im_bin3_2 = grey2bin(im_grey3_2)
    # im3_3 = np.array(plt.imread('3_3_label.png'))
    # im_grey3_3 = rgb2gray(im3_3)
    # im_bin3_3 = grey2bin(im_grey3_3)
    # im3_4 = np.array(plt.imread('3_4_label.png'))
    # im_grey3_4 = rgb2gray(im3_4)
    # im_bin3_4 = grey2bin(im_grey3_4)
    # im3_5 = np.array(plt.imread('3_5_label.png'))
    # im_grey3_5 = rgb2gray(im3_5)
    # im_bin3_5 = grey2bin(im_grey3_5)
    # image3 = np.concatenate((im_grey3_2,im_grey3_3, im_grey3_4,im_grey3_5),axis=1)
    # image = np.concatenate((image2,image3))

    # image = np.concatenate((im_bin2_2, im_bin2_3, im_bin2_4), axis=1)
    # image_small = image[0:4000, 3000:12000]

    # cv2.imwrite("./Road_image_small.png", image_small * 255)

    im_grey_map = np.array(plt.imread('Road_image_small.png'))
    im_bin_map = grey2bin(im_grey_map)
    im_bin_map_pool = cv2.resize(im_bin_map,(360,160))

    # im_small = cv2.resize(im_bin, (200, 200))
    # # ,interpolation=cv2.INTER_AREA)
    # im_small = grey2bin(im_small)
    #image_small
    plt.imshow(im_bin_map_pool)
    plt.show()
