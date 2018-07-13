#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8MultiArray
import time

def talker():
    data_list = Int8MultiArray()
    pub = rospy.Publisher('contl_data',Int8MultiArray, queue_size=2)
    rospy.init_node('talker', anonymous=True)
    while not rospy.is_shutdown():
        data_list.data = [5,5]
        pub.publish(data_list)
        time.sleep(1)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
