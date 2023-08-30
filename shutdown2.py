#!/usr/bin/python
# coding:utf-8
# shutdown/.py
import time
import RPi.GPIO as GPIO
import os
PIN = 9
# import serial


GPIO.setmode(GPIO.BCM)
#GPIO 5pinを入力モードとし、pull up設定とします
GPIO.setup(PIN,GPIO.IN,pull_up_down=GPIO.PUD_UP)

while True:
    GPIO.wait_for_edge(PIN, GPIO.FALLING)
    sw_counter = 0

    while True:
        sw_status = GPIO.input(PIN)
        if sw_status == 0:
            sw_counter = sw_counter + 1
            if sw_counter >= 50:
                print("長押し！")

                os.system("sudo shutdown -h now")
                break
        else:
                print("まだ短い！")
                break

        time.sleep(0.01)
    print(sw_counter)