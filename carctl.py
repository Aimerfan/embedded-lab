import time

from RPi import GPIO


class CarCtl():
    # map to pinouts
    LF_PIN_1 = 0
    LF_PIN_2 = 1
    LB_PIN_1 = 2
    LB_PIN_2 = 3
    RF_PIN_1 = 4
    RF_PIN_2 = 5
    RB_PIN_1 = 6
    RB_PIN_2 = 7


    def __init__(self, *args):
        if len(args) == 0:
            self.pinouts = [26, 19, 13, 6, 12, 16, 20, 21]
        else:
            self.pinouts = args
        
        GPIO.setmode(GPIO.BCM)
        for pin in range(8):
            GPIO.setup(self.pinouts[pin], GPIO.OUT, initial=GPIO.LOW)
    

    def test(self, sec=10):
        try:
            print('Foward')
            GPIO.output(self.pinouts[CarCtl.LF_PIN_1], GPIO.HIGH)
            GPIO.output(self.pinouts[CarCtl.LF_PIN_2], GPIO.LOW)
            GPIO.output(self.pinouts[CarCtl.LB_PIN_1], GPIO.HIGH)
            GPIO.output(self.pinouts[CarCtl.LB_PIN_2], GPIO.LOW)
            GPIO.output(self.pinouts[CarCtl.RF_PIN_1], GPIO.HIGH)
            GPIO.output(self.pinouts[CarCtl.RF_PIN_2], GPIO.LOW)
            GPIO.output(self.pinouts[CarCtl.RB_PIN_1], GPIO.HIGH)
            GPIO.output(self.pinouts[CarCtl.RB_PIN_2], GPIO.LOW)
            time.sleep(sec)
        except KeyboardInterrupt:
            print('Bye')
        finally:
            GPIO.cleanup()


DEBUG = True
if __name__ == '__main__' and DEBUG == True:
    c = CarCtl()
    c.test(100)
    