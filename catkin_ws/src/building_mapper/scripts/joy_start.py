#!/usr/bin/env python

# joy_trigger_start.py
# Use joystick input to launch exploration nodes in jackal
# Intro to Robotics - EE5900 - Spring 2017
#          Assignment #5

#       Project #5 Group #3
#         Ian (Team Lead)
#            Phillip
#            Akhil
#
# /blueooth_teleop/joy
# sensor_msgs/Joy
#
# std_msgs/Header header
#   uint32 seq
#   time stamp
#   string frame_id
# float32[] axes
# int32[] buttons
#
# axes: [ lft - l/r, lft - up/down, L2 (1/-1), rgt - l/r, rgt - u/d, R2 (1/-1)]
# buttons: [ x, circle, sq, tri, L1, R1, share, options, play, L3, R3, DL, DR, DU, DD]
#

import rospy
import roslaunch
import sys
import time
import os
import wall_avoid
import jackal_move
from sensor_msgs.msg import Joy

class joy_control(object):

    def __init__(self):

        #self.argin = arg
        runtime = 10
        rate = rospy.Rate(5)
        rospy.Subscriber("/bluetooth_teleop/joy", Joy, self.joy_callback)

        self.ready = False
        self.trigger = False
        self.explore_mode = 0
        rospy.loginfo("In start")
        while not rospy.is_shutdown():
            if self.trigger == True:

                if self.ready == True:
                    #
                    # package ='rosbag'
                    # executable ='record'
                    #
                    # node = roslaunch.core.Node(package, executable, args=str(os.path.dirname(os.path.realpath(__file__)))[:-7]+"rosbag/rosbag.bag")
                    #
                    # launch = roslaunch.scriptapi.ROSLaunch()
                    # launch.start()
                    #
                    # rosbag_process = launch.launch(node)
                    #
                    if explore_mode == 0:
                        # run = wall_avoid.WallAvoid(runtime)
                        package = 'building_mapper'
                        executable = 'wall_avoid.py'
                    else:
                        # run = jackal_move.discrete_movement(runtime)
                        package = 'building_mapper'
                        executable = 'jackal_move.py'
                    launch = roslaunch.scriptapi.ROSLaunch()
                    launch.start()
                    explore_process = launch.launch(node)
                else:
                    rospy.loginfo("Stopping rosbag record")
                    # rosbag_process.stop()
                    explore_process.stop()
                self.trigger = False
            #rate.sleep()

    def joy_callback(self, data):

        x, circ, sq, tri, L1, R1, share, options, p4, L3, R3, DL, DR, DU, DD = data.buttons
        llr, lud, L2, rlr, rud, R2 = data.axes

        if (x == 1) and (L2 < 0.9) and (R2 < 0.9) and (self.ready == False):
            rospy.loginfo("Controller code received, commencing exploration protocol...")
            self.trigger = True
            self.ready = True

        if (tri == 1) and (L2 < 0.9) and (R2 < 0.9) and (self.ready == True):
            rospy.loginfo("Controller code received, terminating exploration protocol...")
            self.trigger = True
            self.ready = False

        if (sq == 1) and (L2 < 0.9) and (R2 < 0.9) and (self.explore_mode == 1):
            rospy.loginfo("Switching to wall_avoid algorithm...")
            self.explore_mode = 0

        if (circ == 1) and (L2 < 0.9) and (R2 < 0.9) and (self.explore_mode == 0):
            rospy.loginfo("Switching to jackal_move algorithm...")
            self.explore_mode = 1
        # rospy.sleep(1)


if __name__ == "__main__":
    #  if len(sys.argv) > 1:
    #      x = sys.argv[1]
    #
    #  else:
    #      x = "0"  #no input, use default startup

    try:
        rospy.init_node("joy_start", anonymous=False)
        run = joy_control()  #read in joystick input
    except rospy.ROSInterruptException:
        rospy.loginfo("joy_start node terminated.")
