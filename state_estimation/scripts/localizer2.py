#!/usr/bin/env python
from __future__ import division
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import general_kalmanfilter2 as kalmanfilter
import numpy as np
from sympy.matrices import *
import time
import math

startingtime = 0

# Sonars:
AA = 0
sonarF_val = 0.0
sonarFL_val = 0.0
sonarFR_val = 0.0
sonarL_val = 0.0
sonarR_val = 0.0
givenvel = 0
# IMU:
imuRoll = 0.0 # orientation
imuPitch = 0.0
imuYaw = 0.0
imuAngVelX = 0.0 # angular velocity
imuAngVelY = 0.0
imuAngVelZ = 0.0
imuLinAccX = 0.0 # linear acceleration
imuLinAccY = 0.0
imuLinAccZ = 0.0
# estimated parameters
estimRoll = 0.0 # always zero value in 2D navigation
estimPitch = 0.0 # always zero value in 2D navigation
estimYaw = 0.0

ekf_estimation_msg = Odometry()
ekf_estimation_msg.header.frame_id = "odom"
ekf_estimation_msg.child_frame_id = "chassis"

mean2 = 0.148
mean0 = 0
lenAccels = 10
kX = Matrix([0.45, 0.45, -math.pi / 2])
X = Matrix([0.45, 0.45, -math.pi / 2])
prevX = Matrix([0.45, 0.45, -math.pi / 2])
P = zeros(3)
prevV = 0
prevprevV = 0
prevAcc = 0
timestamp = 0
prevtimestamp = 0
accErr = 0
odomX = 0
odomY = 0
odomA = 0
odomVx = 0
odomVy = 0
prevOdomV = 0
start_angle = -math.pi / 2
last10 = []
accels = []
fhandle = file("/home/ubuntu/catkin_ws/src/state_estimation/scripts/xy", 'w')
fhandle1 = file("/home/ubuntu/catkin_ws/src/state_estimation/scripts/angle", 'w')
fhandle2 = file("/home/ubuntu/catkin_ws/src/state_estimation/scripts/dist", "w")

def send_velocity():
    global X, P, imuLinAccX, last10, prevOdomV, odomA, prevV, prevAcc, accErr, prevtimestamp, AA, prevprevV, accels, givenvel, prevX, timestamp, kX
    if time.time() - startingtime > 65:
        fhandle.close()
        fhandle1.close()
    if givenvel > 0.1:
        velocity = mean2
    else:
        velocity = mean0
    imuLinAccX2 = imuLinAccX
    imuAngVelZ2 = imuAngVelZ
    sonars = [sonarR_val, sonarFR_val, sonarF_val, sonarFL_val, sonarL_val, kalmanfilter.norm((imuYaw + start_angle) % (2 * np.pi))]
    ekf_pub = rospy.Publisher('/ekf_estimation', Odometry, queue_size=1)
    ekf_estimation_msg.header.seq += 1
    ekf_estimation_msg.header.stamp = rospy.Time.now()
    
    if timestamp == prevtimestamp:
        return None
    dt = timestamp - prevtimestamp #compute dt for the kalman filter
    prevtimestamp = timestamp
    kX, P, X = kalmanfilter.update(kX, P, dt, sonars, givenvel, imuAngVelZ2) # call kalman filter
    if abs(X[0]) > 0.65 or abs(X[1]) > 0.65: # if the car is outside the walls, then use the previous position
        X[0] = prevX[0]
        X[1] = prevX[1]
    prevX = X
    
    fhandle.write("{};{}\n".format(X[0], X[1]))
    fhandle1.write("{}\n".format(X[2]))
    fhandle2.write("{}\n".format(math.sqrt((X[0]+0.45)**2+(X[1]+0.45)**2)))

    """
    END
    """

    estimYaw = float(X[2]) # orientation to be estimated (-pi,pi]
    # position to be estimated
    ekf_estimation_msg.pose.pose.position.x = float(X[0])
    ekf_estimation_msg.pose.pose.position.y = float(X[1])
    ekf_estimation_msg.pose.pose.position.z = 0.0
    # RPY to quaternion
    quaternion = quaternion_from_euler(estimRoll, estimPitch, estimYaw)
    ekf_estimation_msg.pose.pose.orientation.x = quaternion[0]
    ekf_estimation_msg.pose.pose.orientation.y = quaternion[1]
    ekf_estimation_msg.pose.pose.orientation.z = estimYaw
    ekf_estimation_msg.pose.pose.orientation.w = quaternion[3]
    # velocities to be estimated
    ekf_estimation_msg.twist.twist.linear.x = velocity * np.cos(float(X[2])) # x-linear velocity to be estimated
    ekf_estimation_msg.twist.twist.linear.y = velocity * np.sin(float(X[2])) # y-linear velocity to be estimated
    ekf_estimation_msg.twist.twist.linear.z = 0.0 # always zero value in 2D navigation
    ekf_estimation_msg.twist.twist.angular.x = 0.0 # always zero value in 2D navigation
    ekf_estimation_msg.twist.twist.angular.y = 0.0 # always zero value in 2D navigation
    ekf_estimation_msg.twist.twist.angular.z = imuAngVelZ # angular velocity to be estimated

    ekf_pub.publish(ekf_estimation_msg)

def sonarFrontCallback(msg):
    global sonarF_val
    sonarF_val = msg.range;
    send_velocity()

def sonarFrontLeftCallback(msg):
    global sonarFL_val
    sonarFL_val = msg.range

def sonarFrontRightCallback(msg):
    global sonarFR_val
    sonarFR_val = msg.range

def sonarLeftCallback(msg):
    global sonarL_val
    sonarL_val = msg.range

def sonarRightCallback(msg):
    global sonarR_val
    sonarR_val = msg.range

def imuCallback(msg):
    global imuAngVelX, imuAngVelY, imuAngVelZ, imuLinAccX, imuLinAccY, imuLinAccZ, imuRoll, imuPitch, imuYaw, timestamp, prevtimestamp
    # orientation:: quaternion to RPY (rool, pitch, yaw)
    orientation_q = msg.orientation
    #orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    imuYaw = orientation_q.w
    # angular velocity
    imuAngVelX = msg.angular_velocity.x
    imuAngVelY = msg.angular_velocity.y
    imuAngVelZ = msg.angular_velocity.z

    # linear acceleration
    imuLinAccX = msg.linear_acceleration.x
    imuLinAccY = msg.linear_acceleration.y
    imuLinAccZ = msg.linear_acceleration.z

    t = msg.header.stamp.secs + msg.header.stamp.nsecs * (10 ** (-9))
    if timestamp == 0 and prevtimestamp == 0:
        prevtimestamp = t
    timestamp = t

def cmdvelfun(msg):
    global givenvel
    givenvel = msg.linear.x

def follower_py():
    rospy.init_node('localizer_node', anonymous=True)
    rospy.Subscriber("sonar_front", Range, sonarFrontCallback)
    rospy.Subscriber("sonar_front_left", Range, sonarFrontLeftCallback)
    rospy.Subscriber("sonar_front_right", Range, sonarFrontRightCallback)
    rospy.Subscriber("sonar_left", Range, sonarLeftCallback)
    rospy.Subscriber("sonar_right", Range, sonarRightCallback)
    rospy.Subscriber("imu_data", Imu, imuCallback)
    rospy.Subscriber("cmd_vel", Twist, cmdvelfun)
    print "Ready"
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    try:
        startingtime = time.time()
        #Testing our function
        follower_py()
    except rospy.ROSInterruptException:
        print "Done"
        fhandle.close()
        fhandle1.close()
