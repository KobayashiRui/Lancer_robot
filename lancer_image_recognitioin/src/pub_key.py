#!/usr/bin/env python

import time
import rospy
from std_msgs.msg import Int16
import pygame
from pygame.locals import *
import sys
 
pygame.init()
SCREEN_SIZE = (100,100)
 
pygame.init()
screen = pygame.display.set_mode(SCREEN_SIZE)
pygame.display.set_caption(u"event")

pub_data = Int16()
pub = rospy.Publisher('key_test',Int16,queue_size=1)
rospy.init_node('topic_publisher')
rate = rospy.Rate(2)

while not rospy.is_shutdown():
    for event in pygame.event.get():
        if event.type == QUIT: sys.exit()
        if event.type == KEYDOWN:
            if event.key == K_ESCAPE:
                sys.exit()
            if event.key == K_LEFT:
                print("left")
            if event.key == K_RIGHT:
                print("right")
            if event.key == K_UP:
                pub_data.data = 10
                pub.publish(pub_data)
            if event.key == K_DOWN:
                pub_data.data = 0
                pub.publish(pub_data)
    time.sleep(0.1)
