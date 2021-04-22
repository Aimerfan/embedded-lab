import math
import os
import sys
import time

import smbus2

from emblab import *

LCD_init()

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
    print("<--- Gyroscope data --->")
    gyro_xout = read_word_2c(0x43)
    gyro_xscaled = gyro_xout / 131
    gyro_yout = read_word_2c(0x45)
    gyro_yscaled = gyro_yout / 131
    gyro_zout = read_word_2c(0x47)
    gyro_zscaled = gyro_zout / 131
    print("X-out: %6d  scales: %f" % (gyro_xout, gyro_xscaled))
    print("Y-out: %6d  scales: %f" % (gyro_yout, gyro_yscaled))
    print("Z-out: %6d  scales: %f" % (gyro_zout, gyro_zscaled))
    

    print("<--- Accelerometer data --->")
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

    # show on LCD
    nowtime = time.localtime(time.time())
    line11 = time.strftime("%H:%M:%S", nowtime)
    LCD_print(line11, 0, 0)

    line12 = "X: %.1f" % (gyro_xscaled)
    LCD_print(line12, 0, 9)

    line21 = "Y: %.1f" % (gyro_yscaled)
    LCD_print(line21, 1, 0)

    line22 = "Z: %.1f" % (gyro_zscaled)
    LCD_print(line22, 1, 9)

    line3 = "X-rot: {:.6f}".format(x_rota)
    LCD_print(line3, 2, 0)

    line4 = "Y-rot: {:.6f}".format(y_rota)
    LCD_print(line4, 3, 0)

    time.sleep(0.1)
    os.system('clear')
