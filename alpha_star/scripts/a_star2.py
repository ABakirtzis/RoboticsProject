#!/usr/bin/env python
from __future__ import division
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt
import numpy as np
import pdcon
import time
import linemath
import curvemath
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
import math
from sensor_msgs.msg import Imu
import time


globalstarttime = time.time()
# Sonars:
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
X = [0.45,0.45,-np.pi/2] # x, y, theta
count_state_0 = 0
state = 0
ready_to_write = True
angular_velocity_limit = 2

#planning
start = (0.45, 0.45)
end = (0.30, -0.55)
startang_robot = -np.pi/2
obstacles = [((0.30, 0.15), (-0.30, -0.15))]


cur,obs,obs_s = find_curve(start = start, end = end, obstacles = obstacles, smoothing_range = 0.8)
pd = pdcon.pd_controller(setpoint = 0, kp = 10, kd = 12)
startang_curve = np.arctan2(cur[1][1] - cur[0][1], cur[1][0] - cur[0][0])
startangle = linemath.norm(startang_curve - startang_robot)
fhandle = file("/home/ubuntu/catkin_ws/src/alpha_star/scripts/info", 'w')
pd_turn = pdcon.pd_controller(setpoint = startangle, kp = 2, kd = 0.1)

def send_velocity():
    global sonarF_val
    global sonarFL_val
    global sonarFR_val
    global sonarL_val
    global sonarR_val
    global pd0, pd, state, substate, state2start, startangle, ready_to_write
    global sonarF, sonarL, sonarFL
    global imuYaw
    global state
    global count_state_0
    global X
    x,y,theta = X
    velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    velocity = Twist()
    if time.time() - globalstarttime > 40 and ready_to_write:
        state = 2
        ready_to_write=False
    
    if state == 0:
        #fhandle.write("theta: {}\n".format(theta))
        #fhandle.write("imuyaw {}\n".format(imuYaw))
        velocity.angular.z =  -min(max(pd_turn.pd_out(imuYaw)[0], -2.8), 2.8)
        velocity.linear.x = 0
        #fhandle.write("diafora {}\n".format(linemath.norm(imuYaw - startangle)))
        if abs(linemath.norm(imuYaw - startangle)) < 0.17:
            count_state_0 += 1
            if count_state_0 >= 5:
                velocity.angular.z = 0
                state = 0
                
    elif state == 1:
        try:
            fhandle.write("{};{} {} ".format(x,y,((x - end[0]) ** 2 + (y - end[1]) ** 2) ** (1/2.)))
        except:
            pass
        begin_time = time.time()
        
        if ((x - end[0]) ** 2 + (y - end[1]) ** 2) ** (1/2.) < 0.12:
            state = 2
        m = 1000
        ind = 0
        for i in range(len(curve)):
            if my_dist(cur[i], (x,y)) < m:
                m = my_dist(cur[i], (x,y))
                ind = i
        if ind == 0:
            p1 = cur[ind]
            p2 = cur[ind+1]
        else:
            p2 = cur[ind]
            p1 = cur[ind-1]
        if linemath.is_it_left(p1, p2, (x,y)):
            m = -m
        temp = pd.pd_out(m)[0]
        w = min(max(temp, -angular_velocity_limit), angular_velocity_limit)
        points_ahead = int(curve_lookahead / size * resolution)
        if points_ahead + ind + 1 < len(cur):
            p3 = cur[ind + points_ahead]
            p4 = cur[ind + points_ahead + 1]
            a1 = np.arctan2(p2[1] - p1[1], p2[0] - p1[0])
            a2 = np.arctan2(p4[1] - p3[1], p4[0] - p3[0])
            diffa = linemath.norm(a2 - a1)
            angout, kpa, kda = pda.pd_out(diffa)
            w += angout
        fhandle.write("pda_out: {} kpa: {} kda: {}\n".format(angout, kpa, kda))
        velocity.angular.z =  min(max(w, -angular_velocity_limit), angular_velocity_limit)
        velocity.linear.x = 0.2

        
    elif state == 2:
        ready_to_write = False
        velocity.linear.x = 0
        velocity.angular.z = 0
        velocity_pub.publish(velocity)
        fhandle.close()
        with open("/home/ubuntu/catkin_ws/src/alpha_star/scripts/info", 'r') as fhandle2:
            z = fhandle2.read().split('\n')[:-1]
            x1 = [float(i.split(' ')[0].split(';')[0]) for i in z]
            y1 = [float(i.split(' ')[0].split(';')[1]) for i in z]
            
        plt.plot([i[0] for i in obs], [i[1] for i in obs], 'o')
        plt.plot([i[0] for i in obs_s], [i[1] for i in obs_s], 'o')
        plt.plot([i[0] for i in cur], [i[1] for i in cur], 'o')
        plt.plot(x1, y1, 'o')
        plt.title("kp: {}, kd: {}".format(pd.kp, pd.kd))
        plt.xlim(-0.75, 0.75)
        plt.ylim(-0.75, 0.75)
        with open("/home/ubuntu/catkin_ws/src/alpha_star/scripts/fignum", 'r') as fhandle2:
            fignum = int(fhandle2.read().strip())
            print fignum
        with open("/home/ubuntu/catkin_ws/src/alpha_star/scripts/fignum", 'w') as fhandle2:
            fhandle2.write("{}\n".format(fignum + 1))
        plt.savefig("/home/ubuntu/catkin_ws/src/alpha_star/scripts/astartest{}.png".format(fignum))
        state = 3
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

def imuCallback(msg):
    global imuAngVelX, imuAngVelY, imuAngVelZ, imuLinAccX, imuLinAccY, imuLinAccZ, imuRoll, imuPitch, imuYaw, timestamp, prevtimestamp
    # orientation:: quaternion to RPY (rool, pitch, yaw)
    orientation_q = msg.orientation
    #orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    #(imuRoll, imuPitch, imuYaw) = euler_from_quaternion(orientation_list)
    imuYaw = orientation_q.w
    # angular velocity
    imuAngVelX = msg.angular_velocity.x
    imuAngVelY = msg.angular_velocity.y
    imuAngVelZ = msg.angular_velocity.z

    # linear acceleration
    imuLinAccX = msg.linear_acceleration.x
    imuLinAccY = msg.linear_acceleration.y
    imuLinAccZ = msg.linear_acceleration.z


def kalmanCallback(msg):

    global X
    X[0] = msg.pose.pose.position.x
    X[1] = msg.pose.pose.position.y
    X[2] = msg.pose.pose.orientation.z


def astar_py():
    # Starts a new node
    rospy.init_node('astar_node', anonymous=True)
    rospy.Subscriber("sonar_front_left", Range, sonarFrontLeftCallback)
    rospy.Subscriber("sonar_front_right", Range, sonarFrontRightCallback)
    rospy.Subscriber("sonar_left", Range, sonarLeftCallback)
    rospy.Subscriber("sonar_right", Range, sonarRightCallback)
    rospy.Subscriber("sonar_front", Range, sonarFrontCallback)
    rospy.Subscriber("imu_data", Imu, imuCallback)
    rospy.Subscriber("ekf_estimation", Odometry, kalmanCallback)

    while not rospy.is_shutdown():
        rospy.spin()


if __name__ == '__main__':
    try:
        time.sleep(20)
        astar_py()
    except rospy.ROSInterruptException: pass
