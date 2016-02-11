#!/usr/bin/env python
import smbus
import time
import rospy
from geometry_msgs.msg import Accel

# Get I2C bus
bus = smbus.SMBus(1)

def initialize_imu():
        # MMA8452Q address, 0x1D(29)
        # Select Control register, 0x2A(42)
        #		0x00(00)	StandBy mode
        
        bus.write_byte_data(0x1D, 0x2A, 0x00)
        
        # MMA8452Q address, 0x1D(29)
        # Select Control register, 0x2A(42)
        #		0x01(01)	Active mode
        
        bus.write_byte_data(0x1D, 0x2A, 0x01)

        # MMA8452Q address, 0x1C(28)
        # Select Configuration register, 0x0E(14)
        #		0x00(00)	Set range to +/- 2g

        bus.write_byte_data(0x1D, 0x0E, 0x00)

        
def get_acc():

        # MMA8452Q address, 0x1D(29)
        # Read data back from 0x00(0), 7 bytes
        # Status register, X-Axis MSB, X-Axis LSB, Y-Axis MSB, Y-Axis LSB, Z-Axis MSB, Z-Axis LSB
        
        data = bus.read_i2c_block_data(0x1D, 0x00, 7)
        
        # Convert the data
        xAccl = (data[1] * 256 + data[2]) / 16
        if xAccl > 2047 :
	        xAccl -= 4096
        xAccl *= 2 * 9.81 / 2048.

        yAccl = (data[3] * 256 + data[4]) / 16
        if yAccl > 2047 :
	        yAccl -= 4096
        yAccl *= 2 * 9.81 / 2048.

        zAccl = (data[5] * 256 + data[6]) / 16
        if zAccl > 2047 :
	        zAccl -= 4096
        zAccl *= 2 * 9.81 / 2048.

        return (xAccl, yAccl, zAccl)

def accel_publisher():
        pub = rospy.Publisher("imu_data", Accel, queue_size = 10)
        rospy.init_node("MMA8452Q_IMU", anonymous = True)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
                message = Accel()
                x,y,z = get_acc()
                message.linear.x = x
                message.linear.y = y
                message.linear.z = z
                pub.publish(message)
                rate.sleep()
        


if __name__ == "__main__":
        initialize_imu()
        try:
                accel_publisher()
        except rospy.ROSInterruptException:
                pass
        
