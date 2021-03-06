#! /usr/bin/env python
import rospy
import time
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8MultiArray

#rospy.init_node('cont_talk', anonymous=True)
stealing = 0
throttle = 0

def callback(data):
    if(data.buttons[3] == 1):
        print("start")
    global stealing
    global throttle
    stealing = round((data.axes[0] + 1) * 50,1)
    throttle = round((data.axes[1] + 1) * 50,1)

def listener():
    pub = rospy.Publisher('contl_data',Int8MultiArray,queue_size=10)
    rospy.init_node('listener', anonymous=True)
    data_list = Int8MultiArray()
    global stealing
    global throttle
    while not rospy.is_shutdown():
        data_list.data = [int(throttle),int(stealing)]
        print(data_list.data)
        pub.publish(data_list)
        time.sleep(0.5)
        rospy.Subscriber("joy",Joy,callback)
    

if __name__ == "__main__":
    listener()

