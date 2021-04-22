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


swt_channel = 0
vrx_channel = 1
vry_channel = 2

try:
    while True:
        vrx_pos = ReadChannel(vrx_channel)
        vry_pos = ReadChannel(vry_channel)
        swt_val = ReadChannel(swt_channel)
        print("X:{:>4d} Y:{:>4d} SW:{:>4d}".format(vrx_pos, vry_pos, swt_val))
        time.sleep(0.25)
except KeyboardInterrupt:
    print('ByeBye')
