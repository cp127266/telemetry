from gpiozero import Servo
from gpiozero import AngularServo
from time import sleep

s = AngularServo(23, min_angle = 0, max_angle = 45)
s.angle = 0.0

x = 0

try:
    while True:
        while x < 90:
            s.angle = x
            sleep(10)
            x += 10
    
except KeyboardInterrupt:
    print("Program stopped")

