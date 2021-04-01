import time

from RPi import GPIO

GPIO.setmode(GPIO.BCM)

LED = 17
LED2 = 27
GPIO.setup(LED, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(LED2, GPIO.OUT, initial = GPIO.HIGH)
time.sleep(1)

try:
    while(True):
        GPIO.output(LED, GPIO.HIGH)
        GPIO.output(LED2, GPIO.LOW)
        time.sleep(1)

        GPIO.output(LED, GPIO.LOW)
        GPIO.output(LED2, GPIO.HIGH)
        time.sleep(1)
finally:
    GPIO.cleanup()
