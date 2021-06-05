import sys
import time
import smbus2
from enum import Enum
from RPi import GPIO

# 系統匯流排設定
sys.modules['smbus'] = smbus2
GPIO.setmode(GPIO.BCM)
# 超音波感測器 pin 腳設定
Trig_Pin = 23
Echo_Pin = 24
GPIO.setup(Trig_Pin, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(Echo_Pin, GPIO.IN)
# 紅外線感測器 pin 腳設定
infrared_pin_left = 17
infrared_pin_right = 27
GPIO.setup(infrared_pin_left, GPIO.IN)
GPIO.setup(infrared_pin_right, GPIO.IN)
# wait for initialize
time.sleep(2)


class CarCtl:
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
        GPIO.setmode(GPIO.BCM)
        if len(args) == 0:
            self.pinouts = [19, 26, 6, 13, 16, 12, 21, 20]
        else:
            self.pinouts = args

        for pin in range(8):
            GPIO.setup(self.pinouts[pin], GPIO.OUT, initial=GPIO.LOW)

    def motor_ctl(self, direction: int, motor: str):
        """
        :param direction: 0:stop, 1:forward, 2:backward
        :param motor: motor alias in string, ex: 'LF'
        :return: None
        """
        motor = motor.upper()
        pin1 = f'{motor}_PIN_1'
        pin2 = f'{motor}_PIN_2'
        if direction == 0:
            GPIO.output(self.pinouts[getattr(CarCtl, pin1)], GPIO.LOW)
            GPIO.output(self.pinouts[getattr(CarCtl, pin2)], GPIO.LOW)
        elif direction == 1:
            GPIO.output(self.pinouts[getattr(CarCtl, pin1)], GPIO.HIGH)
            GPIO.output(self.pinouts[getattr(CarCtl, pin2)], GPIO.LOW)
        elif direction == 2:
            GPIO.output(self.pinouts[getattr(CarCtl, pin1)], GPIO.LOW)
            GPIO.output(self.pinouts[getattr(CarCtl, pin2)], GPIO.HIGH)
        else:
            raise Exception('CarCtl.motor_ctl direction should 0,1,2')

    def stop(self):
        self.motor_ctl(0, 'LF')
        self.motor_ctl(0, 'LB')
        self.motor_ctl(0, 'RF')
        self.motor_ctl(0, 'RB')
    
    def forward(self):
        self.motor_ctl(1, 'LF')
        self.motor_ctl(1, 'LB')
        self.motor_ctl(1, 'RF')
        self.motor_ctl(1, 'RB')
    
    def backward(self):
        self.motor_ctl(2, 'LF')
        self.motor_ctl(2, 'LB')
        self.motor_ctl(2, 'RF')
        self.motor_ctl(2, 'RB')
    
    def right(self):
        self.motor_ctl(1, 'LF')
        self.motor_ctl(1, 'LB')
        self.motor_ctl(2, 'RF')
        self.motor_ctl(0, 'RB')
    
    def left(self):
        self.motor_ctl(2, 'LF')
        self.motor_ctl(0, 'LB')
        self.motor_ctl(1, 'RF')
        self.motor_ctl(1, 'RB')


def get_distance(current_tempature=25):
    def sonar_speed(t):
        return 331.6 + 0.6 * t

    # 發射超音波脈衝
    GPIO.output(Trig_Pin, GPIO.HIGH)
    # wait at least 10 microsecond
    time.sleep(0.000100)
    GPIO.output(Trig_Pin, GPIO.LOW)

    # 計算距離
    while not GPIO.input(Echo_Pin):
        pass
    t1 = time.time()
    while GPIO.input(Echo_Pin):
        pass
    t2 = time.time()

    return (t2 - t1) * sonar_speed(current_tempature) * 100 / 2


def car_driver():
    class Drive(Enum):
        STOP = 0
        FORWARD = 1
        RIGHT = 2
        LEFT = 3

    car = CarCtl()
    noise_window = 2
    status_counter = 0
    curt_status = Drive.STOP
    prev_status = Drive.FORWARD

    car.forward()
    dicision = Drive.FORWARD.name
    while True:
        try:
            distance = get_distance()
            block_left = GPIO.input(infrared_pin_left)
            block_right = GPIO.input(infrared_pin_right)
            print(f'dis: {round(distance, 2)}, left: {block_left}, right: {block_right}')

            if distance <= 20:
                dicision = 'obstacle!!!!!'
                car.stop()
                while get_distance() <= 20:
                    pass
                dicision = Drive.FORWARD.name
                car.forward()
            # 皆在白線上, 直走
            elif block_left == 0 and block_right == 0:
                curt_status = Drive.FORWARD
            # 左循跡超出白線, 右轉
            elif block_left != 0 and block_right == 0:
                curt_status = Drive.RIGHT
            # 右循跡超出白線, 左轉
            elif block_left == 0 and block_right != 0:
                curt_status = Drive.LEFT
            # 皆在黑線上, 停止
            else:
                curt_status = Drive.STOP

            if status_counter != 0:
                if curt_status == prev_status:
                    status_counter += 1
                    if status_counter >= noise_window:
                        if curt_status == Drive.STOP:
                            car.stop()
                        elif curt_status == Drive.FORWARD:
                            car.forward()
                        elif curt_status == Drive.RIGHT:
                            car.right()
                        elif curt_status == Drive.LEFT:
                            car.left()
                        dicision = curt_status.name
                        status_counter = 0
                else:
                    status_counter = 0
            elif curt_status != prev_status:
                status_counter = 1
            prev_status = curt_status
            print(f'c_stat: {curt_status.name}, p_stat: {prev_status.name}, counter: {status_counter}')
            print(f'dicision: {dicision}\n')

        except RuntimeError as error:
            # Error happen fairly often, DHT's are hard to read, just keep going
            print(error.args[0])
            # time.sleep(2.0)
            continue
            
        except Exception as e:
            raise e


if __name__ == "__main__":

    print('go!')
    try:
        car_driver()
    except KeyboardInterrupt:
        print('ByeBye')
        GPIO.cleanup()
        exit()
