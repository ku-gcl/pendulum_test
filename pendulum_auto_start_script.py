import time
import RPi.GPIO as GPIO
import os
import subprocess

GPIO.cleanup()
GPIO.setmode(GPIO.BCM)
SW1 = 9        # shutdown
SW2 = 10       # control and shutdown

#GPIO 9pin and 10pinを入力モードとし、pull up設定とする
GPIO.setup(SW1, GPIO.IN, pull_up_down=GPIO.PUD_UP)   # shutdown
GPIO.setup(SW2, GPIO.IN, pull_up_down=GPIO.PUD_UP)   # control and shutdown

# C++コードが実行中かどうかを判定するフラグ変数
CODE_EXEC_FLAG = False

print("\n--------------------")
print("wait for edge... At first, press YELLOW button")
GPIO.wait_for_edge(SW2, GPIO.FALLING)

print("\n--------------------")
print("To start pendulum code, press YELLOW button for 2 seconds")
print("To shutdown, press YELLOW and BLACK button for 5 seconds")

while True:
    
    sw1_timer = 0
    sw2_timer = 0

    while True:
        sw1_status = GPIO.input(SW1)
        sw2_status = GPIO.input(SW2)
        
        # start control program (SW1が押されてなくてSW2が押されてる)
        # sw1_status == 1 and sw2_status == 0
        if sw1_status and not sw2_status:
            sw2_timer = sw2_timer + 1
            if sw2_timer >= 20:
                # すでにC++コードが実行されていたら現在の実行を中断する
                if CODE_EXEC_FLAG:
                    ABORT_CMD1 = "ps aux | grep ros | grep -v grep | awk '{ print \"kill -9\", $2 }' | sh"
                    ABORT_CMD2 = "g++ -o /home/ubuntu/PENDULUM_CLEANUP /home/ubuntu/pendulum_project/pendulum_test/cleanup.cpp -lpigpiod_if2 -lrt"
                    ABORT_CMD3 = "sudo /home/ubuntu/PENDULUM_CLEANUP"
                    subprocess.run(ABORT_CMD1, shell=True)
                    subprocess.run(ABORT_CMD2, shell=True)
                    subprocess.run(ABORT_CMD3, shell=True)
                    print("-------Control END------\n")
                    CODE_EXEC_FLAG = False
                    break
                
                # 制御プログラムを開始する
                print("-------Control START------\n")
                # プログラムのコンパイルコマンド
                COMPILE_CMD = "g++ -o /home/ubuntu/PENDULUM \
                                /home/ubuntu/pendulum_project/pendulum_test/inverted_pendulum_without_kalman.cpp \
                                -lpigpiod_if2 -lrt -pthread"
                COMPILE_CMD = "g++ -o /home/ubuntu/PENDULUM /home/ubuntu/pendulum_project/pendulum_test/inverted_pendulum_without_kalman.cpp -lpigpiod_if2 -lrt -pthread"
                # プログラムの実行コマンド
                EXEC_CMD = "sudo /home/ubuntu/PENDULUM"
                subprocess.run(COMPILE_CMD, shell=True)
                # C++コードをバックグラウンドで実行。そのためにsubprocess.Popenを使用
                subprocess.Popen(EXEC_CMD, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                
                print("-------Control Executed------\n")
                CODE_EXEC_FLAG = True
                break
                
        # shutdown
        # sw1_status == 0 and sw2_status == 0
        elif not sw1_status and not sw2_status:
            sw1_timer = sw1_timer + 1
            if sw1_timer >= 50:
                print("-------SHUTDOWN------\n")
                subprocess.run("sudo shutdown -h now", shell=True)
                break

        # (sw1_status==1 and sw2_status==1) or (sw1_status==0 and sw2_status==1)
        else:
            sw1_timer = 0
            sw2_timer = 0
            GPIO.wait_for_edge(SW2, GPIO.FALLING)
            

        time.sleep(0.1)
