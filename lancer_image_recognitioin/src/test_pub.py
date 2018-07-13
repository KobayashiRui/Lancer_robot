#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16MultiArray
import time

def talker():
    data_list = Int16MultiArray()
    pub = rospy.Publisher('Testmessage',Int16MultiArray, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    while not rospy.is_shutdown():
        data_list.data = [3,10]
        pub.publish(data_list)
        time.sleep(5)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
