#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
roslib.load_manifest('lancer_test')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:
    def __init__(self):
        self.image_pub = rospy.Publisher("/image_topic",Image, queue_size=1)
        self.bridge = CvBridge()
        #self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        #RGB表色系からHSV表色系に変換
        #hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        color_min = np.array([150,100,150])
        color_max = np.array([180,255,255])
        #color_mask = cv2.inRange(hsv_image, color_min, color_max)
        #cv_image2  = cv2.bitwise_and(cv_image, cv_image, mask = color_mask)

        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        ret, cv_image2 = cv2.threshold(gray_image, 150, 255, cv2.THRESH_BINARY_INV)
        cv_image3 = cv2.Canny(gray_image, 15.0, 30.0)
        #cv_half_image = cv2.resize(cv_image, (0,0), fx=1, fy=1)
        #cv2.imshow("Origin Image", cv_half_image)
        cv2.imshow("Origin Image", cv_image)
        cv2.imshow("Result Image", cv_image2)
        cv2.imshow("Edge Image", cv_image3)
        cv2.waitKey(2)


def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
