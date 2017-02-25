#!/usr/bin/env python

# publisher + subscriber that reads twist msgs from
# keyboard and tracks movement

# Intro to Robotics - EE5900 - Spring 2017
#          Assignment #5

#       Project #2 Group #2
#         Ian (Team Lead)
#            Philip
#            Akhil
#
# Revision: v1.2

# imports
import rospy
import sys
import time
import roslaunch
import os

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def Callback(data):
     rospy.loginfo("Callback")

# define setup and run routine
def init():
    # create node for listening to twist messages
    rospy.init_node("building_mapper")

    # subscribe to twist_key
    rospy.Subscriber("/scan", LaserScan, Callback)
    rate = rospy.Rate(10)

    # init Twist msg
    motion = Twist()
    # publish to cmd_vel of the jackal
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    while not rospy.is_shutdown():

        # push Twist msgs
        motion.linear.x = 0.5
        motion.angular.z = 0

        # publish Twist
        pub.publish(motion)
        # pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        rate.sleep()


# standard ros boilerplate
if __name__ == "__main__":
    try:
        init()
    except rospy.ROSInterruptException:
        pass