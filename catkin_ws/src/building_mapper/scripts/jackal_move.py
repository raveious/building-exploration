#!/usr/bin/env python

# publisher + subscriber that reads twist msgs from
# keyboard and tracks movement

# Intro to Robotics - EE5900 - Spring 2017
#          Assignment #5

#       Project #5 Group #3
#         Ian (Team Lead)
#            Philip
#            Akhil
#
# Revision: v1.3

# imports
import rospy
import sys
import time
import roslaunch
import os

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# init Twist msg
motion = Twist()

# motion algorithm in Callback routine for Jackal motion
def Callback(data):
    # len(data.ranges) = 180
    rospy.loginfo('left: %f  forward: %f  right: %f'%(data.ranges[-5], data.ranges[90], data.ranges[5]))

    # move forward
    motion.linear.x  = 0.5
    motion.angular.z = 0.0

    # avoid right wall side
    if data.ranges[10] < 1.0 :
        motion.linear.x  = 0.2
        motion.angular.z = 0.2

    # avoid left wall side
    if data.ranges[-10] < 1.0 :
        motion.linear.x  = 0.2
        motion.angular.z = -0.2

    # sharp turn if right side too close
    if data.ranges[2] < 0.5 :
        motion.linear.x  = 0.1
        motion.angular.z = 0.3

    # sharp turn if left side too close
    if data.ranges[-2] < 0.5 :
        motion.linear.x  = 0.1
        motion.angular.z = -0.3

    # slow down if forward wall too close
    if data.ranges[90] < 1.5 :
        motion.linear.x  = 0.3
        motion.angular.z = 0.0

    # stop and turn at forward wall
    if data.ranges[90] < 1.2 :
        if data.ranges[10]<data.ranges[-10]:
            motion.linear.x  = 0.0
            motion.angular.z = 0.2
        else:
            motion.linear.x  = 0.0
            motion.angular.z = -0.2

# define setup and run routine
def init():
    global motion
    # create node for listening to twist messages
    rospy.init_node("building_mapper")

    # subscribe to twist_key
    rospy.Subscriber("/scan", LaserScan, Callback)
    rate = rospy.Rate(5)

    # publish to cmd_vel of the jackal
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    while not rospy.is_shutdown():

        # push Twist msgs to override Callback algorithm
        # motion.linear.x = -0.3
        # motion.angular.z = 0.0

        # publish Twist
        pub.publish(motion)

        rate.sleep()


# standard ros boilerplate
if __name__ == "__main__":
    try:
        init()
    except rospy.ROSInterruptException:
        pass
