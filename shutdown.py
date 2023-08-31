
# coding:utf-8
# shutdown/.py
import time
import RPi.GPIO as GPIO
import os
# import serial
GPIO.cleanup()
GPIO.setmode(GPIO.BCM)
pin1 = 9        # shutdown
pin2 = 10       # control and shutdown


#GPIO 5pinを入力モードとし、pull up設定とします
GPIO.setup(pin1, GPIO.IN, pull_up_down=GPIO.PUD_UP)   # shutdown
GPIO.setup(pin2, GPIO.IN, pull_up_down=GPIO.PUD_UP)   # control and shutdown

print("begin")

while True:
    print("loop start")
    GPIO.wait_for_edge(pin2, GPIO.FALLING)
    sw_counter1 = 0
    sw_counter2 = 0
    
    print("loop end")

    while True:
        sw_status1 = GPIO.input(pin1)
        sw_status2 = GPIO.input(pin2)
        
        # print("loop loop")
        
        # control (pin1が押されてなくてpin2が押されてる)
        if sw_status1 == 1 and sw_status2 == 0:
            sw_counter2 = sw_counter2 + 1
            if sw_counter2 >= 20:
                print("セイギョカイシ")
                # os.system("sudo ")
                break
                
        # shutdown
        elif sw_status1 == 0 and sw_status2 == 0:
            sw_counter1 = sw_counter1 + 1
            if sw_counter1 >= 50:
                print("オヤスミ")
                os.system("sudo shutdown -h now")
                break

        # pin1: ON, pin2: OFF / pin1: OFF, pin2: ON
        else:
            sw_counter1 = 0
            sw_counter2 = 0
            print("チラ")
            GPIO.wait_for_edge(pin2, GPIO.FALLING)
            

        time.sleep(0.1)
        
    print(sw_counter1)
    print(sw_counter2)