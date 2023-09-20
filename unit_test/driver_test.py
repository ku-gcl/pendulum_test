import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BCM)
GPIO.setup(5,GPIO.OUT)  #IN2
GPIO.setup(6,GPIO.OUT)  #IN1
GPIO.setup(12,GPIO.OUT) #PWM
pi = GPIO.PWM ( 12, 1000 )
pi.start ( 0 )

try:
    while True:
        duty = 50 
        GPIO.output(5,GPIO.HIGH) 
        GPIO.output(6,GPIO.LOW) 
        pi.ChangeDutyCycle( duty )

except KeyboardInterrupt:
    pass
