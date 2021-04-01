import time
from RPi import GPIO

Buzzer_pin = 18
GPIO.setmode(GPIO.BCM)
GPIO.setup(Buzzer_pin, GPIO.OUT)
buzzer = GPIO.PWM(Buzzer_pin, 440)
buzzer.start(50)

tone = {
    0:None, 1:261, 2:293, 3:329, 4:349, 5:392, 6:440, 7:493
}

littlestar = [1,1,5,5,6,6,5,4,4,3,3,2,2,1]
beat = [1,1,1,1,1,1,2,1,1,1,1,1,1,2]

try:
    for i, j in zip(littlestar, beat):
        buzzer.ChangeFrequency(tone[i])
        time.sleep(j * 0.5)

except KeyboardInterrupt:
    print('stop')

finally:
    buzzer.stop()
    GPIO.output(Buzzer_pin, GPIO.HIGH)
    GPIO.cleanup()
