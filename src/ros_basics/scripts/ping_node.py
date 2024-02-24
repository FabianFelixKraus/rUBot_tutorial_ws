#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import random

def create_random_string():
    if random.random() > 0.5:
        return "ping"
    else:
        return "incorrect string"

def ping_node_publisher():
    rospy.init_node("ping_node", anonymous=True)
    pub = rospy.Publisher("/ping", String, queue_size=10)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        msg = String()
        msg.data = create_random_string()
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        ping_node_publisher()
    except rospy.ROSInterruptException:
        pass