
from RPi import GPIO
import time

GPIO.setmode(GPIO.BCM)

EN = 25

GPIO.setup(en, GPIO.OUT)

p = GPIO.PWM(EN, 1000)

p.ChangeDutyCycle(50)
