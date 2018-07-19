#!/usr/bin/env python
import time
import rospy
from std_msgs.msg import Int16MultiArray
import pygame
from pygame.locals import *
import sys
 
pygame.init()
SCREEN_SIZE = (150,150)
 
pygame.init()
screen = pygame.display.set_mode(SCREEN_SIZE)
pygame.display.set_caption(u"event")

data_list = Int16MultiArray()
pub = rospy.Publisher('Testmessage',Int16MultiArray,queue_size=1)
rospy.init_node('talker', anonymous=True)
forward_data  = 50
stealing_data = 50

while not rospy.is_shutdown():
    for event in pygame.event.get():
        if event.type == QUIT: sys.exit()
        if event.type == KEYDOWN:
            if event.key == K_ESCAPE:
                sys.exit()
            if event.key == K_LEFT:
                forward_data  = 50
                stealing_data = 55 
                data_list.data = [forward_data, stealing_data]
                pub.publish(data_list)
            if event.key == K_RIGHT:
                forward_data  = 50
                stealing_data = 45 
                data_list.data = [forward_data, stealing_data]
                pub.publish(data_list)
            if event.key == K_UP:
                forward_data  = 45
                stealing_data = 50 
                data_list.data = [forward_data, stealing_data]
                pub.publish(data_list)
            if event.key == K_DOWN:
                forward_data  = 50
                stealing_data = 50 
                data_list.data = [forward_data, stealing_data]
                pub.publish(data_list)
    time.sleep(0.1)
