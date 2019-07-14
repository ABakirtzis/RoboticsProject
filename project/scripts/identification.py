#!/usr/bin/env python
from __future__ import division
import rospy
from sensor_msgs.msg import Range
import sys

measurements = []

def sub(msg):
    measurements.append(float(msg.range))
    if len(measurements) == 1000:
        m = sum(measurements) / len(measurements)
        std = (sum([(i - m) ** 2 for i in measurements]) / (len(measurements) - 1)) ** (1/2)
        print "std = {}".format(std)


def main(topic):
    rospy.init_node("sampling_node", anonymous = True)
    rospy.Subscriber(topic, Range, sub)
    rospy.spin()

if __name__ == "__main__":
    main(sys.argv[1])
    
