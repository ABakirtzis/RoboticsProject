#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import time
from sensor_msgs.msg import Imu

start_time = time.time()
velocity_pub = 0
fhandle = file("/home/ubuntu/catkin_ws/src/project/scripts/angvel", 'w')
imuAngVelX, imuAngVelY, imuAngVelZ, imuLinAccX, imuLinAccY, imuLinAccZ, imuRoll, imuPitch, imuYaw, timestamp, prevtimestamp = 0,0,0,0,0,0,0,0,0,0,0

def send_velocity():
        global velocity_pub
        velocity = Twist()
	ctime = time.time()
        if (ctime - start_time) < 30:
	        velocity.angular.z = 2.5
                if (ctime - start_time) > 2:
                        fhandle.write("{}\n".format(imuAngVelZ))
	else:
                fhandle.close()
		velocity.angular.z = 0
	velocity_pub.publish(velocity)

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

    t = msg.header.stamp.secs + msg.header.stamp.nsecs * (10 ** (-9))
    if timestamp == 0 and prevtimestamp == 0:
        prevtimestamp = t
    timestamp = t
    send_velocity()
    
def walker_py():
        global velocity_pub
	rospy.init_node('walker_node', anonymous = True)
        rospy.Subscriber("imu_data", Imu, imuCallback)
	velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)

        while not rospy.is_shutdown():
                rospy.spin()

if __name__ ==  '__main__':
        try:
                walker_py()
	except rospy.ROSInterruptException: pass
