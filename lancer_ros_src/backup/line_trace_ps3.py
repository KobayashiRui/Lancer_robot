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
throttle = 50
flag = 0 #0=>ps3controler 1=>linetrace

class image_converter:
    def __init__(self):
        self.image_pub = rospy.Publisher("/contl_data",Int8MultiArray, queue_size=1)
        self.bridge = CvBridge()
        #self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)
        self.image_sub_0 = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback,call_back_args=0)
        self.image_sub_1 = rospy.Subscriber("/joy",Joy,self.callback,call_back_args=1)

    def callback(self,data,joy_data):
        global stealing
        global throttle
        global flag
        data_list = Int8MultiArray()
        if(data.buttons[3] == 1): #start=>linetrace
            flag = 1
        elif(data.buttons[0] == 1): #select=>ps3control
            flag = 0
  
        if(flag == 0):
            data_ste = [round((data.axes[0] -1)* -50,1)] #round is 四捨五入
            stealing = np.clip(data_ste, 35, 65)[0]
            data_thr = [round((data.axes[1] -1)* -50,1)]
            throttle = np.clip(data_thr, 35, 65)[0]
        elif(flag == 1):
            throttle = 45
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print(e)
    
            gray_image = cv2.cvtColor(cv_image[105:119,0:159] ,cv2.COLOR_BGR2GRAY)
            ret, cv_image2 = cv2.threshold(gray_image, 150, 255, cv2.THRESH_BINARY_INV)
            thresh = 210
            max_pixel = 255
            ret, img_dst = cv2.threshold(gray_image, thresh, max_pixel, cv2.THRESH_BINARY)
            img_binary = img_dst /255
            line_data = np.sum(img_binary, axis=0)
            line_data_left  = line_data[0:((line_data.shape[0]/2)-1)].sum()
            line_data_right = line_data[(line_data.shape[0]/2):(line_data.shape[0]-1)].sum()
            cont_data = [float(line_data_left) - float(line_data_right)]
            cont_data = np.clip(cont_data, -50, 50)[0]
            cont_data = (cont_data -50) * -1
            print(cont_data)
            stealing = cont_data
    
            cv2.imshow("img_dst",img_dst)
            cv2.waitKey(2)

        data_list.data = [int(throttle),int(stealing)]
        self.image_pub.publish(data_list)


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
