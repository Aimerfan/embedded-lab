import RPi.GPIO as GPIO
import time

from emblab import *

infrared_pin = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(infrared_pin, GPIO.IN)

LCD_init()

def detect(detect, b_time, nb_time):
    if detect:
        line2 = ' D time: {:2} s'.format(b_time)
        line3 = 'ND time: {:2} s'.format(nb_time)
        line4 = 'Black    '
    else:
        line2 = ' D time: {:2} s'.format(b_time)
        line3 = 'ND time: {:2} s'.format(nb_time)
        line4 = 'Non-Black'
    LCD_print(line2, 1)
    LCD_print(line3, 2)
    LCD_print(line4, 3)

b_time = 0
nb_time = 0
try:
    while True:
        nowtime = time.localtime(time.time())
        line1 = time.strftime("%Y/%m/%d %H:%M:%S", nowtime)
        LCD_print(line1, 0)

        block = GPIO.input(infrared_pin)
        if block:
            b_time += 1 
            detect(block, b_time, nb_time)
        else:
            nb_time += 1
            detect(block, b_time, nb_time)
        print(block)

        # print(GPIO.input(infrared_pin))
        time.sleep(0.95)
except KeyboardInterrupt:
    print('Bye')
finally:
    GPIO.cleanup()
