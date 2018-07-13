#-*- coding:utf-8 -*-
"""
2値化を行う
"""
import cv2
import numpy as np
from matplotlib import pyplot as plt

img = cv2.imread('./image_raw_screenshot_24.07.20182.png')
img = cv2.resize(img,(160,120))

#グレースケール
img_gray = cv2.cvtColor(img[105:119,0:159], cv2.COLOR_RGB2GRAY)

histr = cv2.calcHist([img_gray],[0],None,[256],[0,256])
plt.plot(histr,color = 'k')
plt.show()

#二値化
thresh=210 #pixelが100より大きいと1=白とする
max_pixel = 255
ret, img_dst = cv2.threshold(img_gray,thresh,max_pixel, cv2.THRESH_BINARY)
img_binary = img_dst /255
line_data = np.sum(img_binary, axis=0)
line_data_left  = line_data[0:((line_data.shape[0]/2)-1)].sum()
line_data_right = line_data[(line_data.shape[0]/2):(line_data.shape[0]-1)].sum()
print("left :{}  right:{}".format(line_data_left,line_data_right))
cont_data = [(line_data_left - line_data_right)]
cont_data = np.clip(cont_data, -50,50)[0]
cont_data = (cont_data - 50) * -1
print(cont_data)
#cv2.imshow("gray_image",img_gray)
#cv2.imshow("binarization_image",img_dst)
#cv2.waitKey(0)
#cv2.destroyAllWindows()
