#!/usr/bin/env python
import smbus
import time
import rospy
from sensor_msgs.msg import Imu
import L3GD20
import LSM303


def initialize_imu():
        L3GD20.init_ang()
        LSM303.init_acc_mag()
        time.sleep(0.5)
        

def accel_publisher():
        pub = rospy.Publisher("imu_data", Imu, queue_size = 10)
        rospy.init_node("IMU_NODE", anonymous = True)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
                message = Imu()
                xacc,yacc,zacc,xmag,ymag,zmag = LSM303.read_data()
                xang, yang, zang = L3GD20.read_data()
                message.linear_acceleration.x = xacc
                message.linear_acceleration.y = yacc
                message.linear_acceleration.z = zacc
                message.angular_velocity.x = xang
                message.angular_velocity.y = yang
                message.angular_velocity.z = zang
                message.orientation.w = zmag # see here is the angle
                t = time.time()
                message.header.stamp.secs = int(t)
                message.header.stamp.nsecs = int((t % 1) * 10 ** 9)
                pub.publish(message)
                rate.sleep()
        


if __name__ == "__main__":
        initialize_imu()
        try:
                accel_publisher()
        except rospy.ROSInterruptException:
                pass
        
