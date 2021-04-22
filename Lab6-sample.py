import math
import os
import sys
import time

import smbus2

sys.modules['smbus'] = smbus2
bus = smbus2.SMBus(1)
#MPU-6050 i2c address
mpu_addr = 0x68

#Power management registers
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c
#Wake up MPU-6050 (it starts with sleep mode)
bus.write_byte_data(mpu_addr,power_mgmt_1, 0)

def read_byte(addr):
    return bus.read_byte_data(mpu_addr, addr)


def read_word(addr):
    high = bus.read_byte_data(mpu_addr, addr)
    low = bus.read_byte_data(mpu_addr, addr+ 1 )
    val = (high << 8) + low 
    return val

def read_word_2c(addr):
    val = read_word(addr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else: 
        return val

def dist(a, b):
    return math.sqrt((a*a)+(b*b))

while True:
    print("<--- Gyroscope(?) data --->")
    gyro_xout = read_word_2c(0x43)
    gyro_xscaled = gyro_xout / 131
    gyro_yout = read_word_2c(0x45)
    gyro_yscaled = gyro_yout / 131
    gyro_zout = read_word_2c(0x47)
    gyro_zscaled = gyro_zout / 131
    print("X-out: %6d  scales: %f" % (gyro_xout, gyro_xscaled))
    print("Y-out: %6d  scales: %f" % (gyro_yout, gyro_yscaled))
    print("Z-out: %6d  scales: %f" % (gyro_zout, gyro_zscaled))
    

    print("<--- Accelerometer(?) data --->")
    accel_xout = read_word_2c(0x3b)
    accel_xscaled = accel_xout / 16384.0
    accel_yout = read_word_2c(0x3d)
    accel_yscaled = accel_yout / 16384.0
    accel_zout = read_word_2c(0x3f)
    accel_zscaled = accel_zout / 16384.0
    print("X-out: %6d  scales: %f" % (accel_xout, accel_xscaled))
    print("Y-out: %6d  scales: %f" % (accel_yout, accel_yscaled))
    print("Z-out: %6d  scales: %f" % (accel_zout, accel_zscaled))

    x_rota = math.degrees(math.atan2(accel_yscaled, dist(accel_xscaled, accel_zscaled)))
    y_rota = -math.degrees(math.atan2(accel_xscaled, dist(accel_yscaled, accel_zscaled)))
    print("X-rotation:", x_rota)
    print("Y-rotation:", y_rota)

    time.sleep(0.1)
    os.system('clear')
