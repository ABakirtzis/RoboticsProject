#!/usr/bin/env python
from __future__ import division
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
import time
import math
import pdcon
import sys

sonarF_val = 3.0; #using 3 as inf
sonarFL_val = 3.0;
sonarFR_val = 3.0;
sonarL_val = 3.0;
sonarR_val = 3.0;
sonarL = [] #lists for smoothing inputs
sonarFL = []
sonarF = []
bound = 0.4
see_bound = 0.3
pd_bound = 0.3
state = 0
substate = 0
state2start = 0

pd0 = pdcon.pd_controller(setpoint = 0.3, kp = 27, kd = 2.5)
pd = pdcon.pd_controller(setpoint = 0.2, kp = 27, kd = 2.5)
def send_velocity():
    global sonarF_val
    global sonarFL_val
    global sonarFR_val
    global sonarL_val
    global sonarR_val
    global pd0, pd, state, substate, state2start
    global sonarF, sonarL, sonarFL

    velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    velocity = Twist()
    ctime = time.time()
    
    velocity.linear.x = 0.0
    velocity.angular.z = 0

    sonarF.append(sonarF_val) #smoothing the sonar inputs
    if len(sonarF) > 3:
        sonarF.pop(0)
    sonarFL.append(sonarFL_val)
    if len(sonarFL) > 3:
        sonarFL.pop(0)
    sonarL.append(sonarL_val)
    if len(sonarL) > 3:
        sonarL.pop(0)
    sonarF_val = sum(sonarF) / len(sonarF)
    sonarFL_val = sum(sonarFL) / len(sonarFL)
    sonarL_val = sum(sonarL) / len(sonarL)

    if sonarF_val < 0.4: # too close to a wall, start the turning state

        state = 2
        state2start = time.time()


    #using a state model
    #state 0: no wall visible, search for a wall moving forward
    #state 1: almost parallel to a wall, apply a pd controller to remain parallel
    #state 2: steep turn ahead, turn right

    if state == 0:
        velocity.linear.x = 0.18
        velocity.angular.z = -0.3
        if sonarL_val <= see_bound:
            state = 1
            velocity.angular.z = 0
        elif sonarR_val <= see_bound + 0.2:
            if substate != 1 :
                pd0.reset(sp = 0.4)
            substate = 1
            pd_val = pd0.pd_out(sonarR_val)
            velocity.angular.z += pd_val
            velocity.angular.z = max(min(velocity.angular.z, 0.65), 0.1)
        elif sonarF_val <= see_bound + 0.1:
            if substate != 2:
                pd0.reset(sp = 0.3)
            substate = 2
            pd_val = pd0.pd_out(sonarF_val)
            velocity.angular.z += pd_val
            velocity.angular.z = min(max(velocity.angular.z, -0.65), -0.1)
        elif sonarFL_val <= see_bound:
            if substate != 3:
                pd0.reset(sp = 0.3)
            substate = 3
            pd_val = pd0.pd_out(sonarFL_val)
            velocity.angular.z += pd_val
            velocity.angular.z = min(max(velocity.angular.z, -0.65), -0.1)
        elif sonarFR_val <= see_bound + 0.2:
            if substate != 4 :
                pd0.reset(sp = 0.4)
            substate = 4
            pd_val = pd0.pd_out(sonarFR_val)
            velocity.angular.z += pd_val
            velocity.angular.z = max(min(velocity.angular.z, 0.65), 0.1)
    elif state == 1:
        # the input to the pd controller is the minimum distance from the wall seen by the left and front left sonar
        # assuming we are parallel to the wall
        velocity.linear.x = 0.18
        if sonarR_val <= 0.1: # just to terminate it if communication to raspberry pi is lost
            state = 3
            sys.exit(0)
        velocity.angular.z = min(max(pd.pd_out(min(sonarL_val, sonarFL_val * math.cos(math.pi/6))), -0.5), 0.5)
    elif state == 2: # turn right

        velocity.linear.x = 0.15
        velocity.angular.z = -0.6
        if time.time() - state2start > 1.3:
            state = 1

    velocity_pub.publish(velocity)


def sonarFrontCallback(msg):
    global sonarF_val
    sonarF_val = msg.range;
    send_velocity()

    
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
    global sonarR_val
    sonarR_val = msg.range;

def follower_py():
    rospy.init_node('follower_node', anonymous=True)
    rospy.Subscriber("sonar_front_left", Range, sonarFrontLeftCallback)
    rospy.Subscriber("sonar_front_right", Range, sonarFrontRightCallback)
    rospy.Subscriber("sonar_left", Range, sonarLeftCallback)
    rospy.Subscriber("sonar_right", Range, sonarRightCallback)
    rospy.Subscriber("sonar_front", Range, sonarFrontCallback)


    while not rospy.is_shutdown():
        rospy.spin()


if __name__ == '__main__':
    try:
        follower_py()
    except rospy.ROSInterruptException: pass
