import RPi.GPIO as GPIO
import time

from emblab import *

infrared_pin = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(infrared_pin, GPIO.IN)

LCD_init()

def detect(detect, time):
    if detect:
        line2 = ' D time: {:2} s'.format(time)
        line3 = 'ND time: {:2} s'.format(0)
        line4 = 'Block    '
    else:
        line2 = ' D time: {:2} s'.format(0)
        line3 = 'ND time: {:2} s'.format(time)
        line4 = 'Non-Block'
    LCD_print(line2, 1)
    LCD_print(line3, 2)
    LCD_print(line4, 3)

prev = -1
acctime = 0
try:
    while True:
        nowtime = time.localtime(time.time())
        line1 = time.strftime("%Y/%m/%d %H:%M:%S", nowtime)
        LCD_print(line1, 0)

        block = GPIO.input(infrared_pin)
        if block == prev:
            detect(block, int(time.time() - acctime))
        else:
            prev = block
            acctime = time.time()
            detect(block, 0)
        print(block)

        # print(GPIO.input(infrared_pin))
        time.sleep(0.95)
except KeyboardInterrupt:
    print('Bye')
finally:
    GPIO.cleanup()
