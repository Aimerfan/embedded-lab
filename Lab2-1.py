import sys

import smbus2
from RPLCD.i2c import CharLCD
sys.modules['smbus'] = smbus2

import time

if __name__ == "__main__":

    lcd = CharLCD('PCF8574', address=0x27, port=1, backlight_enabled=True)
    lcd.clear()
    
    while(True):
        today = time.localtime(time.time())
        line1 = time.strftime("%Y/%m/%d %a", today)
        taipei_time = time.strftime("%H:%M:%S", today)
        today = time.localtime(time.time()+3600)
        tokyo_time = time.strftime("%H:%M:%S", today)
        today = time.gmtime(time.time())
        london_time = time.strftime("%I:%M:%S %p", today)

        lcd.cursor_pos = (0, 3)
        #lcd.write_string("{:^20s}".format(line1))
        lcd.write_string(line1)
        lcd.cursor_pos = (1, 0)
        lcd.write_string("{:>6s}: {}".format("Taipei", taipei_time))
        lcd.cursor_pos = (2, 0)
        lcd.write_string("{:>6s}: {}".format("Tokyo", tokyo_time))
        lcd.cursor_pos = (3, 0)
        lcd.write_string("{:>6s}: {}".format("London", london_time))