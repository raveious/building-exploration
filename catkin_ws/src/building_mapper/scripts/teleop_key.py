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
# Revision: v1.1

# imports
import rospy
import sys
from geometry_msgs.msg import Twist

# define mapping for the keyboard
key_mapping = { 'w': [ 0, 1],
                'a': [-1, 0], 
                's': [ 0, 0],
                'x': [0, -1],
                'd': [1,  0]  }

# define setup and run routine
def init():

    # create node for listening to twist messages
    rospy.init_node("teleop_key")

    # subscribe to twist_key
    rospy.Subscriber("cmd_vel", Twist, twistCallback)
    rate = rospy.Rate(user_rate)

    # init Twist msg
    motion = Twist()

    while not rospy.is_shutdown():

        # get key pressed
        if len(msg.data) == 0 or not key_mapping.has_key(msg.data[0]):
          return # unknown key.
        vel = key_mapping[msg.data[0]]

        # push Twist msgs
        motion.linear.x = vel[1]
        motion.angular.z = vel[0]

        # publish Twist
        pub.publish(motion)
        pub = rospy.Publisher("/jackal_velocity_controller/cmd_vel", Twist, queue_size=10)

        rate.sleep()


# standard ros boilerplate
if __name__ == "__main__":
    try:
        init()
    except rospy.ROSInterruptException:
        pass
