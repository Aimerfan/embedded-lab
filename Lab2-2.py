import sys

import smbus2
from RPLCD.i2c import CharLCD
sys.modules['smbus'] = smbus2

import time

if __name__ == "__main__":

    lcd = CharLCD('PCF8574', address=0x27, port=1, backlight_enabled=True)
    lcd.clear()

    text = "Raspberry"

    while(True):
        for i in range(80):
            lcd.clear()
            lcd.cursor_pos = (int(i/20), i%20)
            lcd.write_string(text)
            time.sleep(0.4)