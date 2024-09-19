import time
import RPi.GPIO as GPIO
import os
import subprocess
import signal

# you should set the location of the bin folder
BASE_DIR = "/home/ubuntu/pendulum_pj/pendulum_test/bin/"
PEN_DIR = os.path.join(BASE_DIR, "PENDULUM")
PEN_CLEAN_DIR = os.path.join(BASE_DIR, "PENDULUM_CLEANUP")

GPIO.cleanup()
GPIO.setmode(GPIO.BCM)

# Pin for the LED (GPIO 27)
LED_Y = 27

#Set 9pin(GPIO) and 10pin(GPIO) to "input mode" and "pullup setting"(input is 0 when pressing the button, otherwise input is 1)
SW1 = 9        # shutdown
SW2 = 10       # control and shutdown
GPIO.setup(SW1, GPIO.IN, pull_up_down=GPIO.PUD_UP)   # shutdown
GPIO.setup(SW2, GPIO.IN, pull_up_down=GPIO.PUD_UP)   # control and shutdown

# Set GPIO 27 (LED_Y) as output
GPIO.setup(LED_Y, GPIO.OUT)

# CODE_EXEC_FLAG is a flag variable
# If pendulum program is executing, CODE_EXEC_FLAG = True
CODE_EXEC_FLAG = False

print("\n--------------------")
print("To start pendulum code, press YELLOW button for 2 seconds")
print("To shutdown, press YELLOW and BLACK button for 2 seconds")

while True:
    sw1_timer = 0
    sw2_timer = 0
    
    # Turn on the LED when entering the while loop
    GPIO.output(LED_Y, GPIO.HIGH)  # Turn on LED_Y

    while True:
        sw1_status = GPIO.input(SW1)
        sw2_status = GPIO.input(SW2)
        
        # start control program (SW1 not pressed and SW2 pressed)
        # sw1_status == 1 and sw2_status == 0
        if sw1_status and not sw2_status:
            sw2_timer = sw2_timer + 1
            # If pendulum program is executed already, kill the program and clean up GPIO
            if CODE_EXEC_FLAG:
                exec_process.kill()
                # Prepare to kill for PENDULUM program(C++) and run the code
                KILL_CMD1 = "sudo killall -9 PENDULUM"
                subprocess.run(KILL_CMD1, shell=True)
                
                # Prepare to execute cleanup script and run the codes
                ABORT_CMD2 = "sudo {}".format(PEN_CLEAN_DIR)
                for i in range(5):
                    subprocess.run(ABORT_CMD2, shell=True)
                
                print("-------Control END------\n")
                CODE_EXEC_FLAG = False
                GPIO.output(LED_Y, GPIO.LOW)  # Turn off LED_Y after control ends
                break
            

            # If sw2 is pressed for 2 sec, Pendulum program is executed
            if sw2_timer >= 20:
                print("-------Control START------\n")
                # C++コードをバックグラウンドで実行。そのためにsubprocess.Popenを使用
                EXEC_CMD = "sudo {}".format(PEN_DIR)
                exec_process = subprocess.Popen(EXEC_CMD, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                # stdout, stderr = exec_process.communicate()
                # print("Standard Output:", stdout.decode())
                # print("Error Output:", stderr.decode())
                
                GPIO.wait_for_edge(SW2, GPIO.RISING)
                time.sleep(1)
                print("-------Control Executed------\n")
                CODE_EXEC_FLAG = True
                break
                
        # shutdown
        # sw1_status == 0 and sw2_status == 0
        elif not sw1_status and not sw2_status:
            sw1_timer = sw1_timer + 1
            if sw1_timer >= 20:
                print("-------SHUTDOWN------\n")
                subprocess.run("sudo shutdown -h now", shell=True)
                break

        # elif not sw1_status and sw2_status:
        #    sw_timer = sw_timer + 1
        #    if sw1_timer >= 10:
        #        COMPILE_CMD1 = "g++ -o /home/ubuntu/PENDULUM /home/ubuntu/pendulum_project/pendulum_test/inverted_pendulum_without_kalman.cpp -lpigpiod_if2 -lrt -pthread"
        #        COMPILE_CMD2 = "g++ -o /home/ubuntu/PENDULUM_CLEANUP /home/ubuntu/pendulum_project/pendulum_test/cleanup.cpp -lpigpiod_if2 -lrt"
        #        subprocess.run(COMPILE_CMD1, shell=True)
        #        subprocess.run(COMPILE_CMD2, shell=True)
        #        break
        
        # (sw1_status==1 and sw2_status==1) or (sw1_status==0 and sw2_status==1)
        else:
            sw1_timer = 0
            sw2_timer = 0
            GPIO.wait_for_edge(SW2, GPIO.FALLING)
            

        time.sleep(0.1)
