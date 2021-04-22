import os
import time
import spidev


from emblab import *


LCD_init()
spi  = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1000000


swt_channel = 0
vrx_channel = 1
vry_channel = 2
x_mid = 539
y_mid = 498

# return sticky bias pos.
# num. of channel means x, y, sw
# x channel range [1-1023], mid(bias=0) = 539
# y channel range [1-1023], mid(bias=0) = 498
# sw channel always 1023 ???
def ReadChannel(channel):
    adc = spi.xfer2([1, (8 + channel) << 4, 0])
    data = ((adc[1] & 3) << 8) + adc[2]
    return data


def direct_calc(pos, direct, speed):
    if direct == speed:
        return [(pos[0]-speed)%4, pos[1]]
    elif direct == 2:
        return [(pos[0]-speed)%4, (pos[1]+speed)%20]
    elif direct == 3:
        return [pos[0], (pos[1]+speed)%20]
    elif direct == 4:
        return [(pos[0]+speed)%4, (pos[1]+speed)%20]
    elif direct == 5:
        return [(pos[0]+speed)%4, pos[1]]
    elif direct == 6:
        return [(pos[0]+speed)%4, (pos[1]-speed)%20]
    elif direct == 7:
        return [pos[0], (pos[1]-speed)%20]
    elif direct == 8:
        return [(pos[0]-speed)%4, (pos[1]-speed)%20]
    else:
        return pos


# pos = origin, direct = [1-8]
def changePos(pos, direct, speed):
    LCD_print(' ', pos[0], pos[1])
    n_pos = direct_calc(pos, direct, speed)
    LCD_print('o', n_pos[0], n_pos[1])
    return n_pos


o_pos = [1, 9]
LCD_clear()
try:
    while True:
        vrx_pos = ReadChannel(vrx_channel)
        vry_pos = ReadChannel(vry_channel)
        # swt_val = ReadChannel(swt_channel)
        # print("X:{:>4d} Y:{:>4d} SW:{:>4d}".format(vrx_pos, vry_pos, swt_val))

        vrx_pos = x_mid - vrx_pos
        vry_pos = y_mid - vry_pos
        if vrx_pos < 10 and vrx_pos > -10:
            vrx_pos = 0
        if vry_pos < 10 and vry_pos > -10:
            vry_pos = 0
        # WARNING!!!: speed 3 unavaiable
        speed = int(((abs(vrx_pos) + abs(vry_pos)) / 2) / 180 + 1)
        print(vrx_pos, vry_pos, speed)
        
        if vrx_pos > 0:
            if vry_pos > 0:
                o_pos = changePos(o_pos, 2, speed)
            elif vry_pos < 0:
                o_pos = changePos(o_pos, 4, speed)
            elif vry_pos == 0:
                o_pos = changePos(o_pos, 3, speed)
        elif vrx_pos < 0:
            if vry_pos > 0:
                o_pos = changePos(o_pos, 8, speed)
            elif vry_pos < 0:
                o_pos = changePos(o_pos, 6, speed)
            elif vry_pos == 0:
                o_pos = changePos(o_pos, 7, speed)
        elif vrx_pos == 0:
            if vry_pos > 0:
                o_pos = changePos(o_pos, 1, speed)
            elif vry_pos < 0:
                o_pos = changePos(o_pos, 5, speed)

        time.sleep(0.25)
except KeyboardInterrupt:
    print('ByeBye')
