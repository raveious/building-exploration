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
# Revision: v1.4

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
    def __init__(self, timeout=None):

        self.motion = Twist()
        self.start_time = time.time()

        self.timeout = None
        if timeout:
            self.timeout = timeout

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
        current_time = time.time()

        if (current_time - self.start_time) > self.timeout:
            rospy.signal_shutdown("Execution timer expired")

        # len(data.ranges) = 180
        rospy.logdebug('left: %f  forward: %f  right: %f'%(data.ranges[-8], data.ranges[90], data.ranges[8]))

        # move forward
        self.motion.linear.x  = 0.5
        self.motion.angular.z = 0.0

        # avoid right wall side
        #if data.ranges[8] < 0.85 :
        #    self.motion.linear.x  = 0.3
        #    self.motion.angular.z = 0.25
        #    rospy.loginfo('soft left turn')

        # avoid left wall side
        #if data.ranges[-8] < 0.85 :
        #    self.motion.linear.x  = 0.3
        #    self.motion.angular.z = -0.25
        #    rospy.loginfo(' soft right turn')

        # sharp turn if right side too close
        #if data.ranges[8] < 0.4 :
        #    self.motion.linear.x  = 0.15
        #    self.motion.angular.z = 0.4
        #    rospy.loginfo('hard left turn')

        # sharp turn if left side too close
        #if data.ranges[-8] < 0.4 :
        #    self.motion.linear.x  = 0.15
        #    self.motion.angular.z = -0.4
        #    rospy.loginfo('  hard right turn')

        # slow down if forward wall too close
        #if data.ranges[90] < 1.5 :
        #    self.motion.linear.x  = 0.3
        #    self.motion.angular.z = 0.0
        #    rospy.loginfo('slow down, wall ahead')

        # stop and turn at forward wall
        #if data.ranges[90] < 1.2 :
            # if right side obstacle is closer than left
        #    if data.ranges[9] < data.ranges[-9]:
        #        self.motion.linear.x  = 0.0
        #        self.motion.angular.z = 0.2
        #        rospy.loginfo('stop and turn left')
        #    else:
        #        self.motion.linear.x  = -0.15
        #        self.motion.angular.z = -0.35
        #        rospy.loginfo(' backup and turn right')

        #rospy.loginfo(str(min(data.ranges)))

        if min(data.ranges[:45]) < 0.7 :
            self.motion.linear.x = 0.2
            self.motion.angular.z = 0.2
            rospy.loginfo('turn left')

        if min(data.ranges[-45:]) < 0.7 :
            self.motion.linear.x = 0.2
            self.motion.angular.z = -0.2
            rospy.loginfo('turn right')

        if min(data.ranges[75:105]) < 0.6 :
            self.motion.linear.x = -0.2
            self.motion.angular.z = -0.3
            rospy.loginfo('backup')

# standard ros boilerplate
if __name__ == "__main__":
    try:
        run = discrete_movement(600) #timeout seconds
    except rospy.ROSInterruptException:
        pass
