#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16MultiArray
import time

def talker():
    flag = 1
    data_list = Int16MultiArray()
    pub = rospy.Publisher('contl_data',Int16MultiArray, queue_size=2)

    rospy.init_node('talker', anonymous=True)
    while not rospy.is_shutdown():
        if(flag ==1):
            data_list.data = [25,-60,-60]
            flag = 0
            pub.publish(data_list)
            time.sleep(2)
        elif(flag==2):
            data_list.data = [-15,0,0]
            flag = 0
            pub.publish(data_list)
            time.sleep(1.0)
        elif(flag==3):
            data_list.data = [20,0,0]
            flag = 0
            pub.publish(data_list)
            time.sleep(1)
        else:
            data_list.data = [0,0,0]
            pub.publish(data_list)
            time.sleep(1.0)
        

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
