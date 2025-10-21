import RPi.GPIO as GPIO
import time
from shifter import Shifter, Bug

# Setup GPIO
GPIO.setmode(GPIO.BCM)
dataPin, latchPin, clockPin = 23, 24, 25
s1, s2, s3 = 17, 27, 22  # Input switches

for pin in [s1, s2, s3]:
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Create shifter and bug
s = Shifter(dataPin, latchPin, clockPin)
bug = Bug(s)

last_s2 = GPIO.input(s2)

try:
    while True:
        # a. Turn bug on/off with s1
        bug.running = GPIO.input(s1) == GPIO.HIGH

        # b. Flip wrap state when s2 changes
        s2_state = GPIO.input(s2)
        if s2_state != last_s2:
            bug.isWrapOn = not bug.isWrapOn
            last_s2 = s2_state

        # c. Increase speed when s3 is on (reduce timestep)
        if GPIO.input(s3) == GPIO.HIGH:
            bug.timestep = 0.1 / 3
        else:
            bug.timestep = 0.1

        # move bug
        bug.move_once()

except KeyboardInterrupt:
    GPIO.cleanup()
