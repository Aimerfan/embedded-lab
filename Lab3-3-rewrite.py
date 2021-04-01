import time
import board
import adafruit_dht

from emblab import *


dhtDevice = adafruit_dht.DHT11(board.D17)

if __name__ == "__main__":

    LCD_init()
    sonar_init()

    prevtime = 0.0
    tempature = 0
    
    while True:
        try:
            nowtime = time.time()
            line1 = time.strftime("%Y/%m/%d %H:%M:%S", time.localtime(nowtime))

            LCD_print(line1, 0)
            
            if nowtime - prevtime > 2:
                prevtime = nowtime

                temperature = dhtDevice.temperature
                humidity = dhtDevice.humidity

                line2 = 'Temp: {:.1f}C {:.1f}F'.format(temperature, temperature * 9 / 5 + 32)
                line3 = 'Humi: {:.1f}% RH'.format(humidity)

                LCD_print(line2, 1)
                LCD_print(line3, 2)

            line4 = 'Distance: {:7.2f} cm'.format(sonar_get_distance(tempature))

            LCD_print(line4, 3)

        except RuntimeError as error:
            # Error happen fairly often, DHT's are hard to read, just keep going
            print(error.args[0])
            # time.sleep(2.0)
            continue

        except Exception as error:
            dhtDevice.exit()
            raise error

        except KeyboardInterrupt:
            print('ByeBye')
            GPIO.cleanup()
            exit()
            
