import RPi.GPIO as GPIO
import time
from shifter import Shifter, Bug

GPIO.setmode(GPIO.BCM)
dataPin, latchPin, clockPin = 23, 24, 25
s1, s2, s3 = 17, 27, 22 

for pin in [s1, s2, s3]:
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

s = Shifter(dataPin, latchPin, clockPin)
bug = Bug(s)

last_s2 = GPIO.input(s2)

try:
    while True:
        bug.running = GPIO.input(s1) == GPIO.HIGH
        s2_state = GPIO.input(s2)
        if s2_state != last_s2:
            bug.isWrapOn = not bug.isWrapOn
            last_s2 = s2_state
        if GPIO.input(s3) == GPIO.HIGH:
            bug.timestep = .1/ 3
        else:
            bug.timestep = .1

        bug.move_once()
except KeyboardInterrupt:
    GPIO.cleanup()
