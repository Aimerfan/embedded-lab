import time
import threading
from mfrc522 import SimpleMFRC522
from RPi import GPIO

from emblab import *

Buzzer_pin = 18
GPIO.setmode(GPIO.BCM)
GPIO.setup(Buzzer_pin, GPIO.OUT)
buzzer = GPIO.PWM(Buzzer_pin, 440)

identity = {
    183415192902: 'D0642135'
}

LCD_lock = threading.Lock()
reader = SimpleMFRC522()

def wait_card():
    while True:
        LCD_lock.acquire()
        LCD_clear()
        LCD_lock.release()

        card_id, card_text = reader.read() # 程式會停在這一行，直到讀到卡片為止
        line2 = "ID:{}".format(card_id)

        if card_id in identity.keys():
            line3 = identity[card_id]
            line4 = "Welcome      "
            buzzer.start(50)
            time.sleep(0.1)
            buzzer.stop()
        else:
            line3 = " " * 20
            line4 = "Access Denied"
            for i in range(3):
                buzzer.start(50)
                time.sleep(0.1)
                buzzer.stop()
                time.sleep(0.2)
        
        LCD_lock.acquire()
        LCD_print(line2, 1)
        LCD_print(line3, 2)
        LCD_print(line4, 3)
        LCD_lock.release()
        time.sleep(0.5)


if __name__ == "__main__":
    LCD_init()
    t = threading.Thread(target = wait_card)
    try:
        print("Please hold a tag near the reader.")
        t.start()
        while True:
            # card_id, card_text = reader.read()
            # print("  ID: {}".format(card_id))
            # print("Text: {}".format(card_text))
            # time.sleep(0.5)

            nowtime = time.time()
            line1 = time.strftime("%Y/%m/%d %H:%M:%S", time.localtime(nowtime))
            LCD_lock.acquire()
            LCD_print(line1, 0)
            LCD_lock.release()
    except KeyboardInterrupt:
        print('ByeBye')
    finally:
        GPIO.cleanup()