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
# Revision: v1.5

# imports
import rospy
import sys
import time
import roslaunch
import os

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# defining class for discrete movement algorithm
class discrete_movement(object):
    # define setup and run routine
    def __init__(self):

        self.motion = Twist()

        # create node for listening to twist messages
        rospy.init_node("building_mapper")

        # subscribe to twist_key
        rospy.Subscriber("/scan", LaserScan, self.Callback)
        rate = rospy.Rate(5)

        # publish to cmd_vel of the jackal
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        while not rospy.is_shutdown():

            # # push Twist msgs to override Callback algorithm
            # self.motion.linear.x = -0.3
            # self.motion.angular.z = 0.0

            # publish Twist
            pub.publish(self.motion)

            rate.sleep()


    # motion algorithm in Callback routine for Jackal motion
    def Callback(self, data):

        # len(data.ranges) = 180
        rospy.logdebug('left: %f  forward: %f  right: %f'%(data.ranges[-8], data.ranges[90], data.ranges[8]))

        # move forward
        self.motion.linear.x  = 0.5
        self.motion.angular.z = 0.0

        avoid right wall side
        if data.ranges[8] < 0.85 :
           self.motion.linear.x  = 0.3
           self.motion.angular.z = 0.25
           rospy.logdebug('soft left turn')

        avoid left wall side
        if data.ranges[-8] < 0.85 :
           self.motion.linear.x  = 0.3
           self.motion.angular.z = -0.25
           rospy.logdebug(' soft right turn')

        sharp turn if right side too close
        if data.ranges[8] < 0.4 :
           self.motion.linear.x  = 0.15
           self.motion.angular.z = 0.4
           rospy.logdebug('hard left turn')

        sharp turn if left side too close
        if data.ranges[-8] < 0.4 :
           self.motion.linear.x  = 0.15
           self.motion.angular.z = -0.4
           rospy.logdebug('  hard right turn')

        slow down if forward wall too close
        if data.ranges[90] < 1.5 :
           self.motion.linear.x  = 0.3
           self.motion.angular.z = 0.0
           rospy.logdebug('slow down, wall ahead')

        stop and turn at forward wall
        if data.ranges[90] < 1.2 :
            if right side obstacle is closer than left
           if data.ranges[9] < data.ranges[-9]:
               self.motion.linear.x  = 0.0
               self.motion.angular.z = 0.2
               rospy.logdebug('stop and turn left')
           else:
               self.motion.linear.x  = -0.15
               self.motion.angular.z = -0.35
               rospy.logdebug(' backup and turn right')


# standard ros boilerplate
if __name__ == "__main__":
    try:
        run = discrete_movement()
    except rospy.ROSInterruptException:
        pass
