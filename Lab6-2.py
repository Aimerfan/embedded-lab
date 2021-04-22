import math
import os
import sys
import time
import threading

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

x_rotate = 0
y_rotate = 0

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

def posture_update():
    while True:
        # print("<--- Accelerometer data --->")
        accel_xout = read_word_2c(0x3b)
        accel_xscaled = accel_xout / 16384.0
        accel_yout = read_word_2c(0x3d)
        accel_yscaled = accel_yout / 16384.0
        accel_zout = read_word_2c(0x3f)
        accel_zscaled = accel_zout / 16384.0
        # print("X-out: %6d  scales: %f" % (accel_xout, accel_xscaled))
        # print("Y-out: %6d  scales: %f" % (accel_yout, accel_yscaled))
        # print("Z-out: %6d  scales: %f" % (accel_zout, accel_zscaled))

        x_rota = math.degrees(math.atan2(accel_yscaled, dist(accel_xscaled, accel_zscaled)))
        y_rota = -math.degrees(math.atan2(accel_xscaled, dist(accel_yscaled, accel_zscaled)))
        # print("X-rotation:", x_rota)
        # print("Y-rotation:", y_rota)

        global x_rotate
        x_rotate = x_rota
        global y_rotate
        y_rotate = y_rota

        time.sleep(0.1)
        # os.system('clear')

# init update thread
updateT = threading.Thread(target = posture_update)
updateT.start()

memap = [
    [' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '],
    [' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '],
    [' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '],
    [' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ']
]
ball_rotate = [1, 9]
x_sens = 15
y_sens = 8

# memap[ball_rotate[0]][ball_rotate[1]] = ' '*[:[ball_rotate[1]]+'#'+' '*[' '*([ball_rotate[1]+2:
memap[ball_rotate[0]][ball_rotate[1]] = '#'
while True:
    # judge award
    x_rota = x_rotate
    y_rota = y_rotate
    if abs(x_rota) > 1:
        if x_rota > 0:
            ball_rotate[1] += 1
            if ball_rotate[1] > 19:
                ball_rotate[1] = 19
        else:
            ball_rotate[1] -= 1
            if ball_rotate[1] < 0:
                ball_rotate[1] = 0
    if abs(y_rota) > 1:
        if y_rota > 0:
            ball_rotate[0] += 1
            if ball_rotate[0] > 3:
                ball_rotate[0] = 3
        else:
            ball_rotate[0] -= 1
            if ball_rotate[0] < 0:
                ball_rotate[0] = 0
    print(ball_rotate)
    memap[ball_rotate[0]][ball_rotate[1]] = '#'

    # print to LCD
    for i in range(4):
        LCD_print(memap[i], i)
    LCD_print('o', ball_rotate[0], ball_rotate[1])

    # delay
    avg_ang = (abs(x_rota) + abs(y_rota)) / 2
    delay = (90-avg_ang) * 4/3 + 0.1
    for i in range(int(delay * 10000)):
        pass
