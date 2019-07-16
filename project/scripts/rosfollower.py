#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
import time
import math
sonarF_val = 3.0;
sonarFL_val = 3.0;
sonarFR_val = 3.0;
sonarL_val = 3.0;
sonarR_val = 3.0;
sonarL = []
sonarFL = []
sonarF = []
bound = 0.4
ztotal = 0

class PID:
    def __init__(self):
        self.Kp = 7
        self.Td = 0.8
        self.curr_error = 0
        self.prev_error = 0
        self.sum_error = 0
        self.prev_error_deriv = 0
        self.curr_error_deriv = 0
        self.control = 0
        self.prev_time = time.time()
        self.curr_time = time.time()

    def update_control(self, current_error, reset_prev=False):

        rospy.loginfo("Error %s", current_error)
        self.curr_time = time.time()
        self.curr_error_deriv = (self.curr_error - self.prev_error) / (self.curr_time - self.prev_time)
        rospy.loginfo("Time %s", self.curr_time - self.prev_time)
        #steering angle = P gain + D gain + I gain
#        if abs(self.curr_error_deriv) < 3 * 10**(-11) and self.curr_error_deriv!= 0:
        p_gain = self.Kp * self.curr_error

        d_gain = self.Td * self.curr_error_deriv
            #PID control
        w = p_gain + d_gain  # = control?
#        else:
        #w = 0.0

        self.control = w

        # update error
        self.prev_error = self.curr_error
        self.curr_error = current_error
        self.prev_error_deriv = self.curr_error_deriv
        self.prev_time = self.curr_time
        #print("control", self.control)
        return self.control

pid = PID()
pid2 = PID()
def send_velocity():
    global sonarF_val
    global sonarFL_val
    global sonarFR_val
    global sonarL_val
    global sonarR_val
    global pid
    global ztotal, sonarF, sonarL, sonarFL

    velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    velocity = Twist()
    ctime = time.time()
    """
    PUT YOUR MAIN CODE HERE
    """
    velocity.linear.x = 0.15

    sonarL.append(sonarL_val)
    sonarFL.append(sonarFL_val)
    sonarF.append(sonarF_val)

    if len(sonarL) > 5:
        sonarL.pop(0)
    if len(sonarF) > 5:
        sonarF.pop(0)
    if len(sonarFL) > 5:
        sonarFL.pop(0)

    if not (len(sonarL) > 1 and (sonarL[-1] - sonarL[-2]) < -bound):
        sonarL_val = sum(sonarL) / len(sonarL)

    if not (len(sonarF) > 1 and (sonarF[-1] - sonarF[-2]) < -bound):
        sonarF_val = sum(sonarF) / len(sonarF)

    if not (len(sonarFL) > 1 and (sonarFL[-1] - sonarFL[-2]) < -bound):
        sonarFL_val = sum(sonarFL) / len(sonarFL)

    if sonarL_val > 0.5:
        if sonarF_val <= 0.5:
            velocity.linear.x = 0.1
            velocity.angular.z = 0.3
        elif sonarR_val <= 0.5:
            velocity.angular.z = 0.3
        elif sonarFR_val <= 0.5:
            velocity.angular.z = 0.5
        else:
            velocity.angular.z = 0
    else:
        velocity.linear.x = 0.1
        z1 = pid.update_control(min(sonarL_val, sonarFL_val - 0.02, sonarF_val -0.4) - 0.1)
    #z2 = pid2.update_control(math.sqrt(sonarL_val**2 + sonarFL_val**2) -1)
        velocity.angular.z = max(min(0.3, z1),-0.3)
        if (velocity.linear.x - abs(z1)) < 0.1:
            velocity.linear.x = 0.1
        else:
            velocity.linear.x = velocity.linear.x - abs(z1)
        #ztotal = ztotal max(min(0.3, z1),-0.3)
        #velocity.angular.z = -z1
#    else:
 #       velocity.angular.z = 0
  #      velocity.linear.x = 0
###    if ztotal >= 70:
#        velocity.angular.z = 0
#        velocity.linear.x = 0
#flag2 = 1
#    velocity.angular.z = -max(min(0.1, (z1 + z2)/2 ),-0.1)
#\    velocity.angular.z = 0
    rospy.loginfo("Velocity x, z: %s, %s", velocity.linear.x, velocity.angular.z)
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
