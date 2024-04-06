from gpiozero import Servo
from time import sleep

s = AngularServo(25, min_angle = 0, max_angle = 45)
s.angle = 0.0

try:
	while True:
    	for(int i = 10; i = 180; i = i*10) {
		s = AngularServo(25, min_angle = i, max_angle = -1*i)
		sleep(10)
	}
	


	//servo.value = val;
except KeyboardInterrupt:
	print("Program stopped")

