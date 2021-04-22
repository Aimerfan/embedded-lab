import time

def LCD_init(addr=0x27):
    import sys

    import smbus2
    from RPLCD.i2c import CharLCD
    sys.modules['smbus'] = smbus2

    global lcd

    lcd = CharLCD('PCF8574', address=addr, port=1, backlight_enabled=True)
    lcd.clear()

def LCD_print(line, row=0, col=0):
    lcd.cursor_pos = (row, col)
    lcd.write_string(line)

def LCD_clear():
    lcd.clear()

Trig_Pin = 0
Echo_Pin = 0

from RPi import GPIO

def sonar_init(tpin=20, epin=21):
    global Trig_Pin
    Trig_Pin = tpin
    global Echo_Pin
    Echo_Pin = epin

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(Trig_Pin, GPIO.OUT, initial = GPIO.LOW)
    GPIO.setup(Echo_Pin, GPIO.IN)
    time.sleep(2)

def sonar_get_distance(tempature=14):
    GPIO.output(Trig_Pin, GPIO.HIGH)
    time.sleep(0.000100) # wait at least 10 microsecond
    GPIO.output(Trig_Pin, GPIO.LOW)
    while not GPIO.input(Echo_Pin):
        pass
    t1 = time.time()
    while GPIO.input(Echo_Pin):
        pass
    t2 = time.time()
    return (t2 - t1) * (331.6 + 0.6 * tempature) * 100 / 2