#!/usr/bin/env python
from __future__ import division
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
import time
import math
import pdcon
import sys

sonarF_val = 3.0;
sonarFL_val = 3.0;
sonarFR_val = 3.0;
sonarL_val = 3.0;
sonarR_val = 3.0;
sonarL = []
sonarFL = []
sonarF = []
bound = 0.4
see_bound = 0.3
pd_bound = 0.3
state = 1
substate = 0

fhandle = file("/home/ubuntu/catkin_ws/src/project/scripts/raw_data", 'w')
pd0 = pdcon.pd_controller(setpoint = 0.3, kp = 2, kd = 0.8)
pd = pdcon.pd_controller(setpoint = 0.2, kp = 27, kd = 2.5)
def send_velocity():
    global sonarF_val
    global sonarFL_val
    global sonarFR_val
    global sonarL_val
    global sonarR_val
    global pd0, pd, state, substate
    global sonarF, sonarL, sonarFL

    velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    velocity = Twist()
    ctime = time.time()
    """
    PUT YOUR MAIN CODE HERE
    """
    velocity.linear.x = 0.0
    velocity.angular.z = 0

    sonarF.append(sonarF_val)
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
    if state == 0:
        print "state 0"
        fhandle.write("state 0\n")
        velocity.linear.x = 0.12
        velocity.angular.z = 0.1
        if sonarL_val <= see_bound:
            print "substate 0"
            fhandle.write("substate 0\n")
            state = 1
            velocity.angular.z = 0
            fhandle.close()
        elif sonarR_val <= see_bound + 0.2:
            if substate != 1 :
                pd0.reset(sp = 0.4)
            print "substate 1"
            fhandle.write("substate 1\n")
            substate = 1
            pd_val = pd0.pd_out(sonarR_val)
            velocity.angular.z += pd_val
            velocity.angular.z = max(min(velocity.angular.z, 0.3), 0.1)
            fhandle.write("PD ret {}\n.".format(pd_val))
        elif sonarF_val <= see_bound + 0.1:
            if substate != 2:
                pd0.reset(sp = 0.3)
            print "substate 2"
            fhandle.write("substate 2\n")
            substate = 2
            pd_val = pd0.pd_out(sonarF_val)
            velocity.angular.z += pd_val
            velocity.angular.z = min(max(velocity.angular.z, -0.3), -0.1)
            fhandle.write("PD ret {}\n.".format(pd_val))
        elif sonarFL_val <= see_bound:
            if substate != 3:
                pd0.reset(sp = 0.3)
            print "substate 3"
            fhandle.write("substate 3\n")
            substate = 3
            pd_val = pd0.pd_out(sonarFL_val)
            velocity.angular.z += pd_val
            velocity.angular.z = min(max(velocity.angular.z, -0.3), -0.1)
            fhandle.write("PD ret {}\n.".format(pd_val))
        elif sonarFR_val <= see_bound + 0.2:
            if substate != 4 :
                pd0.reset(sp = 0.4)
            print "substate 4"
            fhandle.write("substate 4\n")
            substate = 4
            pd_val = pd0.pd_out(sonarFR_val)
            velocity.angular.z += pd_val
            velocity.angular.z = max(min(velocity.angular.z, 0.3), 0.1)
            fhandle.write("PD ret {}\n.".format(pd_val))
    elif state == 1:
        rospy.loginfo("in state 1")
        velocity.linear.x = 0.2
        if sonarF_val / 3 < 0.3:
            velocity.linear.x = 0.15
        if sonarR_val <= 0.1:
            rospy.loginfo("should terminate")
            fhandle.close()
            state = 2
            sys.exit(0)
        #velocity.angular.z = pd.pd_out(min(sonarL_val, sonarFL_val * math.cos(math.pi/6), sonarF_val / 3)) 
        #velocity.angular.z = min(max(pd.pd_out(min(sonarL_val, sonarFL_val * math.cos(math.pi/6), sonarF_val - 0.2)), -0.5), 0.5)
        velocity.angular.z = min(max(pd.pd_out(min(sonarL_val, sonarFL_val * math.cos(math.pi/6))), -0.5), 0.5)
               
            
    #rospy.loginfo("Velocity x, z: %s, %s", velocity.linear.x, velocity.angular.z)
    rospy.loginfo("Right Scan %s", sonarR_val)
    rospy.loginfo("Front Right Scan %s", sonarFR_val)
    rospy.loginfo("Left Scan %s", sonarL_val)
    rospy.loginfo("Front Left Scan %s", sonarFL_val)
    rospy.loginfo("Front Scan %s", sonarF_val)
    
    #fhandle.write("Right Scan {}\n".format( sonarR_val))
    #fhandle.write("Front Right Scan {}\n".format( sonarFR_val))
    fhandle.write("Left Scan {}\n".format( sonarL_val))
    fhandle.write("Front Left Scan {}\n".format( sonarFL_val))
    fhandle.write("Front Scan {}\n".format( sonarF_val))
    fhandle.write("Angular Velocity {}\nLinear Velocity {}\n".format(velocity.angular.z, velocity.linear.x))
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
    # Starts a new node
    rospy.init_node('follower_node', anonymous=True)
    rospy.Subscriber("sonar_front_left", Range, sonarFrontLeftCallback)
    rospy.Subscriber("sonar_front_right", Range, sonarFrontRightCallback)
    rospy.Subscriber("sonar_left", Range, sonarLeftCallback)
    rospy.Subscriber("sonar_right", Range, sonarRightCallback)
    rospy.Subscriber("sonar_front", Range, sonarFrontCallback)


    while not rospy.is_shutdown():
        rospy.spin()
   #     if ztotal>=70:
    #        break

if __name__ == '__main__':
    try:
        #Testing our function
        follower_py()
    except rospy.ROSInterruptException: pass
