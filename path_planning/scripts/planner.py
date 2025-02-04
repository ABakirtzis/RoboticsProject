#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
import pdcon
import math
from sensor_msgs.msg import Imu
import time
#import matplotlib.pyplot as plt
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

start_point = [0.45, 0.45, -math.pi / 2]

end_point = [0.0, -0.45]



target_ang = 0.0
state = 2
count_state_0 = 0
theta = math.atan2(end_point[1]- start_point[1], end_point[0]- start_point[0]) - start_point[2]
pd_turn = pdcon.pd_controller(setpoint = theta, kp = 9, kd = 1.9)

attractive_gain = 80.0
repulsive_gain = 150.0
grid_area_width = 1.5
grid_resolution = 0.075
robot_radious = 0.17
cells_count = int(grid_area_width / grid_resolution)
obstacle = [(0.15 + 0.075, -0.15 - 0.075)]
obstacle1 = (0.1,-0.1)

moves = [[1,0],
         [0,1],
         [-1,0],
         [0,-1],
         [-1,-1],
         [-1,1],
         [1,-1],
         [1,1]]
         

def point_dist(p1, p2):

    return math.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)


def calculate_attractive_potential(x, y):

    sum = 0
    for obst in obstacle:
        sum += 0.5 * attractive_gain * point_dist([x, y], end_point)
    return sum

def calculate_repulsive_potential(x,y):
    sum = 0
    for obst in obstacle:
        dq = point_dist([x,y], obst)

        if dq <= robot_radious:
            if dq < 0.05:
                dq = 0.05
                sum += 0.5 * repulsive_gain * (1.0 / dq - 1.0 / robot_radious)**2
            else:

                sum += 0
    return sum 

def calculate_potential_field_values():

    potential_map = [[0.0 for i in range(0, cells_count)] for j in range(0, cells_count)]

    for x_cell in range(0, cells_count):

        x = x_cell * grid_resolution - 0.75

        for y_cell in range(0, cells_count):

            y = y_cell * grid_resolution - 0.75

            u_att = calculate_attractive_potential(x, y)
            u_rep = calculate_repulsive_potential(x, y)
            u = u_att + u_rep
            potential_map[x_cell][y_cell] = u

    return potential_map


pmap = calculate_potential_field_values()
print "ready 2"
#plt.grid(True)
#plt.axis('equal')
#plt.pcolor(pmap, vmax = 200, cmap=plt.cm.Blues)
#plt.show()

def find_cell(x,y):

    x_cell = int(round((x + 0.75) / grid_resolution))
    y_cell = int(round((y + 0.75) / grid_resolution))

    return x_cell, y_cell

def send_velocity():
    global sonarF_val
    global sonarFL_val
    global sonarFR_val
    global sonarL_val
    global sonarR_val
    global pd0, pd, state, substate, state2start
    global sonarF, sonarL, sonarFL
    global imuYaw
    global state
    global count_state_0
    velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    velocity = Twist()
    """
    PUT YOUR MAIN CODE HERE
    """

    velocity.linear.x = 0.0
    velocity.angular.z = 0

    
    if point_dist(X, end_point) < 0.13:
        state = 3

    # line pose torwards target point
    if state == 0:
        
        velocity.angular.z =  -min(max(pd_turn.pd_out(imuYaw), -2.8), 2.8)
        if abs(imuYaw - theta) < 0.07:
            count_state_0 += 1
            if count_state_0 >= 5:
                velocity.angular.z = 0
            
                print "fuck"
                state = 2
    # follow the straight line connection x, y to target point
    elif state == 1:
        velocity.linear.x = 0.2
        theta1 = math.atan2(end_point[1]-X[1], end_point[0]-X[0])
        pd_line = pdcon.pd_controller(setpoint = theta1, kp = 12, kd = 1.5)
        velocity.angular.z = -min(max(pd_line.pd_out(X[2]), -0.5), 0.5)

    elif state == 2:
        velocity.linear.x = 0.2
        minp = float("inf")
        mini = 0
        for i in range(0, 8):

            x_cell, y_cell = find_cell(X[0], X[1])
            print "{} {}".format(x_cell,y_cell)
            x_cell += moves[i][0]
            y_cell += moves[i][1]
            print "{} {} {} {}".format(x_cell, y_cell, len(pmap), len(pmap[0]))
            if x_cell >= len(pmap) or y_cell >= len(pmap[0]):

                p = float("inf")
            else:
                
                p = pmap[x_cell][y_cell]

            if minp > p:

                minp = p
                mini = i
                x_choice = x_cell * grid_resolution - 0.75
                y_choice = y_cell * grid_resolution - 0.75
            print "{} {}".format(x_choice, y_choice)
        theta_choice = math.atan2(y_choice-X[1], x_choice-X[0])
                
        pd_turn1 = pdcon.pd_controller(setpoint = theta_choice, kp = 9, kd = 1.9)
        velocity.angular.z =  -min(max(pd_turn1.pd_out(X[2]), -1.5), 1.5)
        
    # target reached
    elif state == 3:
        
        velocity.linear.x = 0.0
        velocity.angular.z = 0.0
   

    
            
               
            
    #rospy.loginfo("Velocity x, z: %s, %s", velocity.linear.x, velocity.angular.z)
   ### rospy.loginfo("Right Scan %s", sonarR_val)
   # rospy.loginfo("Front Right Scan %s", sonarFR_val)
   # rospy.loginfo("Left Scan %s", sonarL_val)
   #### rospy.loginfo("Front Left Scan %s", sonarFL_val)
   # rospy.loginfo("Front Scan %s", sonarF_val)
    rospy.loginfo("Imu Yaw %s", imuYaw)
    rospy.loginfo("Velocity %s", velocity.angular.z)
    rospy.loginfo("Target %s", theta)
    rospy.loginfo("X[2] %s", X[2] * 180 / math.pi)
    #fhandle.write("Right Scan {}\n".format( sonarR_val))
    #fhandle.write("Front Right Scan {}\n".format( sonarFR_val))
    #fhandle.write("Left Scan {}\n".format( sonarL_val))
    #fhandle.write("Front Left Scan {}\n".format( sonarFL_val))
    #fhandle.write("Front Scan {}\n".format( sonarF_val))
    #fhandle.write("Angular Velocity {}\nLinear Velocity {}\n".format(velocity.angular.z, velocity.linear.x))
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

X = [0,0,0]

def kalmanCallback(msg):

    global X

    X[0] = msg.pose.pose.position.x
    X[1] = msg.pose.pose.position.y
    X[2] = msg.pose.pose.orientation.z

    #send_velocity()



def follower_py():
    # Starts a new node
    rospy.init_node('follower_node', anonymous=True)
    rospy.Subscriber("sonar_front_left", Range, sonarFrontLeftCallback)
    rospy.Subscriber("sonar_front_right", Range, sonarFrontRightCallback)
    rospy.Subscriber("sonar_left", Range, sonarLeftCallback)
    rospy.Subscriber("sonar_right", Range, sonarRightCallback)
    rospy.Subscriber("sonar_front", Range, sonarFrontCallback)
    rospy.Subscriber("imu_data", Imu, imuCallback)
    rospy.Subscriber("ekf_estimation", Odometry, kalmanCallback)

    while not rospy.is_shutdown():
        rospy.spin()
   #     if ztotal>=70:
    #        break

if __name__ == '__main__':
    try:
        #Testing our function
        time.sleep(20)
        follower_py()
    except rospy.ROSInterruptException: pass
