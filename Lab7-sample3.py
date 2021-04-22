import RPi.GPIO as GPIO
import time

from emblab import *

SERVO_PIN = 21
FREQUENCY = 50
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)

pwm = GPIO.PWM(SERVO_PIN, FREQUENCY)
pwm.start(0)

def angle2duty(angle=0):
    duty = (0.05 * FREQUENCY) + (0.19 * FREQUENCY * angle / 180)
    return duty

try:
    # while True:
    #     nowtime = time.localtime(time.time())
    #     line1 = time.strftime("%Y/%m/%d %H:%M:%S", nowtime)
    #     LCD_print(line1, 0)

    for angle in range(0, 181, 15):
        dc = angle2duty(angle)
        pwm.ChangeDutyCycle(dc)
        time.sleep(2)

except KeyboardInterrupt:
    print('Bye')
finally:
    pwm.stop()
    GPIO.cleanup()