import os
import time
import spidev

spi  = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1000000


def ReadChannel(channel):
    adc = spi.xfer2([1, (8 + channel) << 4, 0])
    data = ((adc[1] & 3) << 8) + adc[2]
    return data

mq_channel = 4

try:
    while True:
        mq_val = ReadChannel(mq_channel)
        print("MQ:{:>4d}".format(mq_val))
        time.sleep(0.25)
except KeyboardInterrupt:
    print("ByeBye")
