# Distributed with a free-will license.
# Use it any way you want, profit or free, provided it fits in the licenses of its associated works.
# L3GD20
# This code is designed to work with the L3GD20_I2CS I2C Mini Module available from ControlEverything.com.
# https://www.controleverything.com/products

import smbus
import time
from math import pi

# Get I2C bus
bus = smbus.SMBus(1)

def init_ang():
    # L3GD20 address, 0x6B(106)
    # Select Control register1, 0x20(32)
    #		0x0F(15)	Normal mode, X, Y, Z-Axis enabled
    bus.write_byte_data(0x6B, 0x20, 0x0F)
    # L3GD20 address, 0x6B(106)
    # Select Control register4, 0x23(35)
    #		0x30(48)	Continous update, Data LSB at lower address
    #					FSR 2000dps, Self test disabled, 4-wire interface
    bus.write_byte_data(0x6B, 0x23, 0x30)


def read_data():
    # L3GD20 address, 0x6B(106)
    # Read data back from 0x28(40), 2 bytes, X-Axis LSB first
    data0 = bus.read_byte_data(0x6B, 0x28)
    data1 = bus.read_byte_data(0x6B, 0x29)

    # Convert the data
    xGyro = data1 * 256 + data0
    if xGyro > 32767 :
	xGyro -= 65536

    # L3GD20 address, 0x6B(106)
    # Read data back from 0x2A(42), 2 bytes, Y-Axis LSB first
    data0 = bus.read_byte_data(0x6B, 0x2A)
    data1 = bus.read_byte_data(0x6B, 0x2B)

    # Convert the data
    yGyro = data1 * 256 + data0
    if yGyro > 32767 :
	yGyro -= 65536

    # L3GD20 address, 0x6B(106)
    # Read data back from 0x2C(44), 2 bytes, Z-Axis LSB first
    data0 = bus.read_byte_data(0x6B, 0x2C)
    data1 = bus.read_byte_data(0x6B, 0x2D)

    # Convert the data
    zGyro = data1 * 256 + data0
    if zGyro > 32767 :
	zGyro -= 65536

    return (xGyro, yGyro, zGyro * 0.07 * pi / 180)

if __name__ == "__main__":
    init_ang()
    while True:
        print read_data()
        time.sleep(0.1)
