import time
import board
import adafruit_dht

# from RPi import GPIO

# GPIO.cleanup()

dhtDevice = adafruit_dht.DHT11(board.D17)

while True:
    try:
        temperature = dhtDevice.temperature
        humidity = dhtDevice.humidity

        print('T: {:.1f}C, H: {:.1f}%'.format(temperature,humidity))

    except RuntimeError as error:
        #Error happen fairly often,
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

    time.sleep(2.0)
