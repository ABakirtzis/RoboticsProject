#!/usr/bin/env python
from __future__ import division
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

givenvel = 0
realvel = 0
fhandle = file("~/catkin_ws/src/state_estimation/scripts/veldata.txt", 'w+')
c = 0

def cmdvelfun(msg):
    global givenvel
    rospy.loginfo("in cmdvel")
    givenvel = msg.linear.x
    

def odomfun(msg):
    global realvel, c
    rospy.loginfo("in realvel")
    odomVx = msg.twist.twist.linear.x
    odomVy = msg.twist.twist.linear.y
    odomAng = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]
    realvel = (odomVx ** 2 + odomVy ** 2) ** (1/2)
    fhandle.write("{},{}\n".format(givenvel, realvel))
    c+=1
    

def follower_py():
    # Starts a new node
    rospy.init_node('deviation_node', anonymous=True)
    rospy.Subscriber("cmd_vel", Twist, cmdvelfun)
    rospy.Subscriber("odom", Odometry, odomfun)
    print "skata"

    while not rospy.is_shutdown():
        rospy.spin()
    try:
        fhandle.close()
    except:
        pass

if __name__ == '__main__':
    try:
        #Testing our function
        follower_py()
    except rospy.ROSInterruptException:
        fhandle.close()
