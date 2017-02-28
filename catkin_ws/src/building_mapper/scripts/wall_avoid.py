#!/usr/bin/env python
import rospy
import math

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class WallAvoid(object):
    def __init__(self):
        rospy.init_node("WallAvoid")

        self.turnCoef = [((1.5 * x) ** 2 + 8100) / 10000000.0 for x in range(-90, 0)] + [((-1.5 * x) ** 2 - 8100) / 10000000.0 for x in range(0, 91)]
        self.speedCoef = [(-x ** 2 + 8100) / 10000000.0 for x in range(-90,91)]

        rospy.Subscriber("/scan", LaserScan, self.latestScan)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        rospy.loginfo("Ready to get out there and avoid some walls!")
        rospy.logdebug(self.turnCoef)
        rospy.logdebug(self.speedCoef)

        rospy.spin()

    def latestScan(self, data):
        turnVal = 0
        speedVal = 0
        for p in range(0, 181):
            # Inf range return means its over 10m from the LIDAR
            if math.isinf(data.ranges[p]) or math.isnan(data.ranges[p]):
                continue
            # If average is really REALLY close, might want to back up instead
            if sum(data.ranges[75:105]) / len(data.ranges[75:105]) < 0.25:
                speedVal = speedVal - (self.speedCoef[p] * data.ranges[p])
            else:
                speedVal = speedVal + (self.speedCoef[p] * data.ranges[p])

            # Turn away from walls
            turnVal = turnVal + (self.turnCoef[p] * data.ranges[p])

        cmd = Twist()
        cmd.linear.x = min(speedVal, 0.3)
        cmd.angular.z = -turnVal

        rospy.loginfo(cmd)

        self.pub.publish(cmd)

# standard ros boilerplate
if __name__ == "__main__":
    try:
        run = WallAvoid()
    except rospy.ROSInterruptException:
        pass
