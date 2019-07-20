#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import time

start_time = time.time()
velocity_pub = 0

def send_velocity():
        global velocity_pub
        velocity = Twist()
	ctime = time.time()
        if (ctime - start_time) < 3.5:
	        velocity.linear.x = 0.2
	else:
		velocity.linear.x = 0
	velocity_pub.publish(velocity)
	        
def walker_py():
        global velocity_pub
	rospy.init_node('walker_node', anonymous = True)
	velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)

        rate = rospy.timer.Rate(10)
        while not rospy.is_shutdown():
                send_velocity()
                rate.sleep()

if __name__ ==  '__main__':
        try:
                walker_py()
	except rospy.ROSInterruptException: pass
