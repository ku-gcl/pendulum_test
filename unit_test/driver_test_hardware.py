import pigpio
from time import sleep

pi = pigpio.pi()
pi.set_mode(5, pigpio.OUTPUT)
pi.set_mode(6, pigpio.OUTPUT)
pi.set_mode(12, pigpio.OUTPUT)

pi.set_mode(5, pigpio.OUTPUT)
pi.write(5, 1)
pi.hardware_PWM(12, 100, 100000)
sleep(5)