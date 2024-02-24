#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def callback_string(msg):
	if msg.data == "ping":
		rospy.loginfo("ping")
	else:
		rospy.loginfo("Failed!")

def pong_node():
    rospy.init_node('pong_node')
    rospy.Subscriber("/ping", String, callback_string)
    rospy.spin()

if __name__ == '__main__':
    pong_node()