import time
from mfrc522 import SimpleMFRC522
from RPi import GPIO

if __name__ == "__main__":
    reader = SimpleMFRC522()
    try:
        while True:
            print("Please hold a tag near the reader.")
            card_id, card_text = reader.read()
            print("  ID: {}".format(card_id))
            print("Text: {}".format(card_text))
            time.sleep(0.5)
    except KeyboardInterrupt:
        print('ByeBye')
    finally:
        GPIO.cleanup()