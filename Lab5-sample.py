import RPi.GPIO as GPIO
import time

infrared_pin = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(infrared_pin, GPIO.IN)

try:
    while True:
        print(GPIO.input(infrared_pin))
        time.sleep(0.1)
except KeyboardInterrupt:
    print('Bye')
finally:
    GPIO.cleanup()
