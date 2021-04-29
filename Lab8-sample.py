from RPi import GPIO
import time

GPIO.setmode(GPIO.BCM)

LF_PIN_1 = 27
LF_PIN_2 = 22
LB_PIN_1 = 13
LB_PIN_2 = 6
RF_PIN_1 = 12
RF_PIN_2 = 16
RB_PIN_1 = 20
RB_PIN_2 = 21

GPIO.setup(LF_PIN_1, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(LF_PIN_2, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(LB_PIN_1, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(LB_PIN_2, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(RF_PIN_1, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(RF_PIN_2, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(RB_PIN_1, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(RB_PIN_2, GPIO.OUT, initial=GPIO.LOW)

if __name__ == '__main__':
    try:
        print('Foward')
        GPIO.output(LF_PIN_1, GPIO.HIGH)
        GPIO.output(LF_PIN_2, GPIO.LOW)
        GPIO.output(LB_PIN_1, GPIO.HIGH)
        GPIO.output(LB_PIN_2, GPIO.LOW)
        GPIO.output(RF_PIN_1, GPIO.HIGH)
        GPIO.output(RF_PIN_2, GPIO.LOW)
        GPIO.output(RB_PIN_1, GPIO.HIGH)
        GPIO.output(RB_PIN_2, GPIO.LOW)
        time.sleep(100)
    except KeyboardInterrupt:
        print('Bye')
    finally:
        GPIO.cleanup()
