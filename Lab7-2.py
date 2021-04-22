import os
import time
import spidev
from emblab import *

spi  = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1000000


def ReadChannel(channel):
    adc = spi.xfer2([1, (8 + channel) << 4, 0])
    data = ((adc[1] & 3) << 8) + adc[2]
    return data

mq_channel = 4
LCD_init()
try:
    while True:
        nowtime = time.time()
        line1 = time.strftime("%Y/%m/%d %H:%M:%S", time.localtime(nowtime))
        LCD_print(line1, 0)
        
        mq_val = ReadChannel(mq_channel)
        line2 = "MQ:{:>4d}".format(mq_val)
        LCD_print(line2, 1)
        print("MQ:{:>4d}".format(mq_val))
        time.sleep(0.25)
except KeyboardInterrupt:
    print("ByeBye")
