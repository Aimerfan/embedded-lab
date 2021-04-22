# -*- coding: utf-8 -*-
"""
Created on Thu Apr  8 10:19:20 2021

@author: Vivian
"""

import keyboard
from emblab import *
import random

'''
#計算陀螺儀上下左右wsda
w = 
s =
d =
a = 
'''

LCD_init()

xbound = 3
ybound = 19
x1 = 10
y1 = 2
x1_change = 0
y1_change = 0

def our_snake(snake_list):
    for j in snake_list:
        lcd.cursor_pos( j[1], j[0])
        
        
snake_List = []
Length_of_snake = 1

foodx = random.randrange(0, 3)
foody = random.randrange(0, 19)
while True:
    
    
    #方向改變時，改變蛇頭位置
    if keyboard.is_pressed("w"):
        y1_change = snake_block
        x1_change = 0
    elif keyboard.is_pressed("s"):
        y1_change = -snake_block
        x1_change = 0
        
    elif keyboard.is_pressed("d"):
        y1_change = 0
        x1_change = snake_block
        
    elif keyboard.is_pressed("a"):
        y1_change = 0
        x1_change = -snake_block
    
    
        
    x1 += x1_change
    y1 += y1_change
    snake_Head = []
    snake_Head.append(x1)
    snake_Head.append(y1)
    snake_List.append(snake_Head)
    if len(snake_List) > Length_of_snake:
            del snake_List[0]
    
    our_snake(snake_List)
    time.sleep(1)
    if x1 == foodx and y1 == foody:
        foodx = random.randrange(0, xbound)
        foody = random.randrange(0, ybound)
        Length_of_snake += 1
        

    
    if x1 > 20 or y1> 4 or x1 < 0 or y1 < 0 :
        break

line = "{}".format("game_over")
LCD_print(line)