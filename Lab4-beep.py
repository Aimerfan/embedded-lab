import time
from RPi import GPIO

Buzzer_pin = 18
GPIO.setmode(GPIO.BCM)
GPIO.setup(Buzzer_pin, GPIO.OUT)
buzzer = GPIO.PWM(Buzzer_pin, 440)
buzzer.start(50)

try:
    buzzer.ChangeFrequency(1000)
    time.sleep(0.1)
except KeyboardInterrupt:
    print('Stop')
finally:
    buzzer.stop()
    GPIO.output(Buzzer_pin, GPIO.HIGH)
    GPIO.cleanup()