#!/usr/bin/env python
from __future__ import division
import rospy
from sensor_msgs.msg import Imu
import sys

measurements = []

def sub(msg):
    measurements.append(float(msg.angular_velocity.z))
    if len(measurements) == 1000:
        m = sum(measurements) / len(measurements)
        std = (sum([(i - m) ** 2 for i in measurements]) / (len(measurements) - 1)) ** (1/2)
        print "m = {}, std = {}".format(m, std)


def main():
    rospy.init_node("identification", anonymous = True)
    rospy.Subscriber("imu_data", Imu, sub)
    rospy.spin()

if __name__ == "__main__":
    main()
    
