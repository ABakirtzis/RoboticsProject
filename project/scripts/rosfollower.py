#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
import time
import math
import pdcon


sonarF_val = 3.0;
sonarFL_val = 3.0;
sonarFR_val = 3.0;
sonarL_val = 3.0;
sonarR_val = 3.0;
sonarL = []
sonarFL = []
sonarF = []
bound = 0.4
see_bound = 0.5
state = 0

pid = pdcon.pd_controller(setpoint = 0.1, kp = 7, kd = 0.8)
def send_velocity():
    global sonarF_val
    global sonarFL_val
    global sonarFR_val
    global sonarL_val
    global sonarR_val
    global pid, state
    global sonarF, sonarL, sonarFL

    velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    velocity = Twist()
    ctime = time.time()
    """
    PUT YOUR MAIN CODE HERE
    """
    velocity.linear.x = 0
    velocity.angular.z = 0

    if state == 0:
        velocity.linear.x = 0.15
        velocity.angular.z = -0.1
        if sonarF_val <= see_bound or sonarFL_val <= see_bound:
            velocity.angular.z = -0.2
        elif sonarL_val <= see_bound:
            state = 1
            velocity.angular.z = 0
        elif sonarR_val <= see_bound or sonarFR_val <= see_bound:
            velocity.angular.z = 0.2
        
    
    rospy.loginfo("Velocity x, z: %s, %s", velocity.linear.x, velocity.angular.z)
    rospy.loginfo("Right Scan %s", sonarR_val)
    rospy.loginfo("Front Right Scan %s", sonarFR_val)
    rospy.loginfo("Left Scan %s", sonarL_val)
    rospy.loginfo("Front Left Scan %s", sonarFL_val)
    rospy.loginfo("Front Scan %s", sonarF_val)
    velocity_pub.publish(velocity)


def sonarFrontCallback(msg):
    global sonarF_val
    sonarF_val = msg.range;

    send_velocity()
    #find_wall()

def sonarFrontLeftCallback(msg):
    global sonarFL_val
    sonarFL_val = msg.range;

def sonarFrontRightCallback(msg):
    global sonarFR_val
    sonarFR_val = msg.range;

def sonarLeftCallback(msg):
    global sonarL_val
    sonarL_val = msg.range;

def sonarRightCallback(msg):
    global sonarR_vall
    sonarR_val = msg.range;

def follower_py():
    # Starts a new node
    rospy.init_node('follower_node', anonymous=True)
    rospy.Subscriber("sonar_front", Range, sonarFrontCallback)
    rospy.Subscriber("sonar_front_left", Range, sonarFrontLeftCallback)
    rospy.Subscriber("sonar_front_right", Range, sonarFrontRightCallback)
    rospy.Subscriber("sonar_left", Range, sonarLeftCallback)
    rospy.Subscriber("sonar_right", Range, sonarRightCallback)


    while not rospy.is_shutdown():
        rospy.spin()
   #     if ztotal>=70:
    #        break

if __name__ == '__main__':
    try:
        #Testing our function
        follower_py()
    except rospy.ROSInterruptException: pass
