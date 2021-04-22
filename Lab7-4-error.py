# not finish
import os
import time
import spidev
import RPi.GPIO as GPIO

from emblab import *

spi  = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1000000

SERVO_PIN = 17
FREQUENCY = 50
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)

swt_channel = 0
vrx_channel = 1
vry_channel = 2
x_mid = 539
y_mid = 498

pwm = GPIO.PWM(SERVO_PIN, FREQUENCY)
pwm.start(0)

def angle2duty(angle=0):
    duty = (0.05 * FREQUENCY) + (0.19 * FREQUENCY * angle / 180)
    return duty

# return sticky bias pos.
# num. of channel means x, y, sw
# x channel range [1-1023], mid(bias=0) = 539
# y channel range [1-1023], mid(bias=0) = 498
# sw channel always 1023 ???
def ReadChannel(channel):
    adc = spi.xfer2([1, (8 + channel) << 4, 0])
    data = ((adc[1] & 3) << 8) + adc[2]
    return data

try:
    while True:
        angle = 0
        vrx_pos = ReadChannel(vrx_channel)
        vry_pos = ReadChannel(vry_channel)
        # swt_val = ReadChannel(swt_channel)
        # print("X:{:>4d} Y:{:>4d} SW:{:>4d}".format(vrx_pos, vry_pos, swt_val))

        vrx_pos = x_mid - vrx_pos
        vry_pos = y_mid - vry_pos
        
        if vrx_pos < 10 and vrx_pos > -10:
            vrx_pos = 0
        if vry_pos < 10 and vry_pos > -10:
            vry_pos = 0
        
        if vrx_pos > 0:
            if vry_pos > 0:     # 2
                angle = 45
            elif vry_pos < 0:   # 4
                angle = 135
            elif vry_pos == 0:  # 3
                angle = 90
        elif vrx_pos == 0:
            if vry_pos > 0:     # 1
                angle = 0
            elif vry_pos < 0:   # 5
                angle = 180

        print("X:{:>4d} Y:{:>4d} Angle:{:3d}".format(vrx_pos, vry_pos, angle))

        dc = angle2duty(angle)
        pwm.ChangeDutyCycle(dc)

        time.sleep(0.25)
except KeyboardInterrupt:
    print('ByeBye')
