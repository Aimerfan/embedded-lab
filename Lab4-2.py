import time
from RPi import GPIO

Buzzer_pin = 18
GPIO.setmode(GPIO.BCM)
GPIO.setup(Buzzer_pin, GPIO.OUT)
buzzer = GPIO.PWM(Buzzer_pin, 440)
buzzer.start(50)

tone = {
    -2: 220, -1:247, 0:None, 1:261, 2:293, 3:329, 4:349, 5:392, 6:440, 7:493, 8:523, 9:587
}

# mary has a little sheep
# sheep = [3, 2, 1, 2, 3, 3, 3, 3, 2, 2, 2, 2, 3, 5, 5, 0, 3, 2, 1, 2, 3, 3, 3, 3, 2, 2, 3, 2, 1, 1, 1, 0]
# beat = [1,1,1,1,1,1,2,1,1,1,1,1,1,2]

# pui pui car
car = [4, 3, 4, 3, 4, 3, 2, -1, 2, 3, 4, 5, 6, 7, 5, 2, -1, 2, 3, 5, 5, 6, 6, 7, 1, -1, -1, -2, 2, -1, 2, 3, 4, 5, 6, 7, 5]
beat = [1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 1, 1, 2, 2, 4, 1, 1, 2, 2, 1, 1, 2, 2, 4, 2, 2, 2, 2, 1, 1, 2, 2, 1, 1, 2, 2, 4]

try:
    for i, j in zip(car, beat):
        buzzer.ChangeFrequency(tone[i])
        time.sleep(j * 0.2)

except KeyboardInterrupt:
    print('stop')

finally:
    buzzer.stop()
    GPIO.output(Buzzer_pin, GPIO.HIGH)
    GPIO.cleanup()
