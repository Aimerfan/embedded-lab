import sys

import smbus2
from RPLCD.i2c import CharLCD
sys.modules['smbus'] = smbus2

import time
import board
import adafruit_dht

dhtDevice = adafruit_dht.DHT11(board.D17)

if __name__ == "__main__":

    lcd = CharLCD('PCF8574', address=0x27, port=1, backlight_enabled=True)
    lcd.clear()

    prevtime = 0.0
    
    while True:
        try:
            nowtime = time.time()
            line1 = time.strftime("%Y/%m/%d %H:%M:%S", time.localtime(nowtime))

            lcd.cursor_pos = (0, 0)
            lcd.write_string(line1)
            
            if nowtime - prevtime > 2:
                prevtime = nowtime

                temperature = dhtDevice.temperature
                humidity = dhtDevice.humidity

                line2 = 'Temp: {:.1f}C {:.1f}F'.format(temperature, temperature * 9 / 5 + 32)
                line3 = 'Humi: {:.1f}% RH'.format(humidity)

                lcd.cursor_pos = (1, 0)
                lcd.write_string(line2)

                lcd.cursor_pos = (2, 0)
                lcd.write_string(line3)

            # print('T: {:.1f}C, H: {:.1f}%'.format(temperature, humidity))

        except RuntimeError as error:
            # Error happen fairly often, DHT's are hard to read, just keep going
            print(error.args[0])
            time.sleep(2.0)
            continue

        except Exception as error:
            dhtDevice.exit()
            raise error

        except KeyboardInterrupt:
            print('ByeBye')
            # GPIO.cleanup()
            exit()
