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
# IMU:
imuYaw = 0.0

X = [0.45,0.45,-np.pi/2] # x, y, theta
count_state_0 = 0
state = 0
ready_to_write = True
angular_velocity_limit = 2

#planning
start = (0.30, 0.45)
end = (0.2, -0.45)
startang_robot = -np.pi/2
obstacles = [((0.30, 0.15), (-0.30, -0.15))]
resolution = 80
size = 1.5
safety_net = 0.13

cur,obs,obs_s = curvemath.find_curve(start = start, end = end, resolution = resolution, safety_net = safety_net, obstacles = obstacles, smoothing_range = 0.8, plot_ret = True)
startang_curve = np.arctan2(cur[1][1] - cur[0][1], cur[1][0] - cur[0][0])
startangle = linemath.norm(startang_curve - startang_robot)
fhandle = file("/home/ubuntu/catkin_ws/src/alpha_star/scripts/info", 'w')
pd_turn = pdcon.pd_controller(setpoint = startangle, kp = 5, kd = 0.4, f = linemath.norm)
atan_coefficient = 11
esc_pd = False

def send_velocity():
    global pd0, pd, state, substate, state2start, startangle, ready_to_write
    global imuYaw, atan_coefficient
    global state, esc_pd
    global count_state_0
    global X
    x,y,theta = X
    velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    velocity = Twist()
    if time.time() - globalstarttime > 40 and ready_to_write:
        state = 2
        ready_to_write=False
    
    if state == 0:
        velocity.angular.z =  min(max(pd_turn.pd_out(imuYaw), -angular_velocity_limit), angular_velocity_limit)
        if abs(velocity.angular.z) < 0.5:
            velocity.angular.z = 0
        velocity.linear.x = 0
        if abs(linemath.norm(imuYaw - startangle)) < 0.17:
            count_state_0 += 1
            if count_state_0 >= 5:
                velocity.angular.z = 0
                state = 1
                
    elif state == 1:
        try:
            fhandle.write("{};{} {}\n".format(x,y,((x - end[0]) ** 2 + (y - end[1]) ** 2) ** (1/2.)))
        except:
            pass
        begin_time = time.time()
        
        if ((x - end[0]) ** 2 + (y - end[1]) ** 2) ** (1/2.) < 0.05:
            state = 2
        m = 1000
        ind = 0
        for i in range(len(cur)):
            if curvemath.my_dist(cur[i], (x,y)) < m:
                m = curvemath.my_dist(cur[i], (x,y))
                ind = i
        temp = int(0.1 * resolution / size)
        if ind + temp < len(cur):
            p1 = cur[ind + temp]
        else:
            p1 = cur[-1]
        angle_setpoint = linemath.norm(np.arctan2(p1[1] - y, p1[0] - x))
        angle_setpoint += 2 * np.pi
        pd_turn.setpoint = angle_setpoint
        temp = pd_turn.pd_out(linemath.norm(imuYaw + startang_robot) + 2 * np.pi)
        w = min(max(temp, -angular_velocity_limit), angular_velocity_limit)
        velocity.angular.z =  w
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
        plt.plot(x1, y1, '--', linewidth = 2)
        plt.title("C: {}".format(atan_coefficient))
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


def imuCallback(msg):
    global imuAngVelX, imuAngVelY, imuAngVelZ, imuLinAccX, imuLinAccY, imuLinAccZ, imuRoll, imuPitch, imuYaw, timestamp, prevtimestamp

    orientation_q = msg.orientation
    imuYaw = orientation_q.w
    send_velocity()


def kalmanCallback(msg):

    global X
    X[0] = msg.pose.pose.position.x
    X[1] = msg.pose.pose.position.y
    X[2] = msg.pose.pose.orientation.z


def astar_py():
    # Starts a new node
    rospy.init_node('astar_node', anonymous=True)
    rospy.Subscriber("imu_data", Imu, imuCallback)
    rospy.Subscriber("ekf_estimation", Odometry, kalmanCallback, queue_size=1, tcp_nodelay=True)

    while not rospy.is_shutdown():
        rospy.spin()


if __name__ == '__main__':
    try:
        time.sleep(20)
        astar_py()
    except rospy.ROSInterruptException: pass
