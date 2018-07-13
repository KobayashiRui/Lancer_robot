#!/usr/bin/env python
# -*- coding: utf-8 -*-

#import roslib
#roslib.load_manifest('lancer_test')
import sys
import rospy
import time
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8MultiArray
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt

stealing = 50
throttle = 40

class image_converter:
    def __init__(self):
        self.image_pub = rospy.Publisher("/contl_data",Int8MultiArray, queue_size=1)
        self.bridge = CvBridge()
        #self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)

    def callback(self,data):
        global stealing
        global throttle
        data_list = Int8MultiArray()
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        #RGB表色系からHSV表色系に変換
        #hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        #hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        #color_min = np.array([150,100,150])
        #color_max = np.array([180,255,255])
        #color_mask = cv2.inRange(hsv_image, color_min, color_max)
        #cv_image2  = cv2.bitwise_and(cv_image, cv_image, mask = color_mask)

        gray_image = cv2.cvtColor(cv_image[105:119,0:159] ,cv2.COLOR_BGR2GRAY)
        #histr = cv2.calcHist([gray_image],[0],None,[256],[0,256])
        #plt.plot(histr,color = 'k')
        #plt.show()
        ret, cv_image2 = cv2.threshold(gray_image, 150, 255, cv2.THRESH_BINARY_INV)
        thresh = 210
        max_pixel = 255
        ret, img_dst = cv2.threshold(gray_image, thresh, max_pixel, cv2.THRESH_BINARY)
        img_binary = img_dst /255
        line_data = np.sum(img_binary, axis=0)
        line_data_left  = line_data[0:((line_data.shape[0]/2)-1)].sum()
        line_data_right = line_data[(line_data.shape[0]/2):(line_data.shape[0]-1)].sum()
        #print("left :{} right:{}".format(line_data_left,line_data_right))
        cont_data = [float(line_data_left) - float(line_data_right)]
        #print(cont_data)
        cont_data = np.clip(cont_data, -50, 50)[0]
        cont_data = (cont_data -50) * -1
        print(cont_data)
        stealing = cont_data
        data_list.data = [int(throttle),int(stealing)]
        self.image_pub.publish(data_list)
        cv2.imshow("img_dst",img_dst)
        cv2.waitKey(2)


def main():
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
