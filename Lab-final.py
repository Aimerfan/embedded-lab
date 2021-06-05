import sys

import smbus2
sys.modules['smbus'] = smbus2

import time
import board
from RPi import GPIO

Trig_Pin = 23
Echo_Pin = 24
infrared_pin_left = 17
infrared_pin_right = 27
GPIO.setmode(GPIO.BCM)
GPIO.setup(Trig_Pin, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(Echo_Pin, GPIO.IN)
GPIO.setup(infrared_pin_left, GPIO.IN)
GPIO.setup(infrared_pin_right, GPIO.IN)

time.sleep(2)

def sonar_speed(tempature):
    return 331.6 + 0.6 * tempature

def get_distance(tempature):
    GPIO.output(Trig_Pin, GPIO.HIGH)
    time.sleep(0.000100) # wait at least 10 microsecond
    GPIO.output(Trig_Pin, GPIO.LOW)
    while not GPIO.input(Echo_Pin):
        pass
    t1 = time.time()
    while GPIO.input(Echo_Pin):
        pass
    t2 = time.time()
    return (t2 - t1) * sonar_speed(tempature) * 100 / 2

if __name__ == "__main__":

    prevtime = 0.0
    tempature = 25

    print('go!')
    while True:
        try:
            distance = get_distance(tempature)
            print('distance: ', distance)

            blockL = GPIO.input(infrared_pin_left)
            blockR = GPIO.input(infrared_pin_right)
            if blockL:
                print('left: Black')
            else:
                print('left: Non-Black')
            print('left: ', blockL)
            if blockR:
                print('right: Black')
            else:
                print('right: Non-Black')
            print('right: ', blockR)

            time.sleep(0.95)

            if distance <= 25:
                # stop
                pass

        except RuntimeError as error:
            # Error happen fairly often, DHT's are hard to read, just keep going
            print(error.args[0])
            # time.sleep(2.0)
            continue

        except KeyboardInterrupt:
            print('ByeBye')
            GPIO.cleanup()
            exit()
            
