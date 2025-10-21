import random
import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)

class Shifter:
    def __init__(self, serialPin, latchPin, clockPin):
        self.serialPin = serialPin
        self.latchPin = latchPin
        self.clockPin = clockPin

        GPIO.setup(self.serialPin, GPIO.OUT)
        GPIO.setup(self.latchPin, GPIO.OUT, initial=0) 
        GPIO.setup(self.clockPin, GPIO.OUT, initial=0)
        
    def __ping(self, a):
        GPIO.output(a,1)
        time.sleep(0)
        GPIO.output(a,0)

    def shiftByte(self, b):
        for i in range(8):
            GPIO.output(self.serialPin, b & (1<<i))
            self.__ping(self.clockPin)
        self.__ping(self.latchPin)

class Bug:
    def __init__(self, __shifter, timestep = 0.1, multiple = 3, isWrapOn = False):
        self.timestep = timestep
        self.multiple = multiple
        self.isWrapOn = isWrapOn
        self.__shifter = __shifter
        self.running = False
    
    def move_once(self):
        """Move the bug one step if it's running."""
        if not self.running:
            return

        pattern = 1 << self.multiple
        self.__shifter.shiftByte(pattern)
        time.sleep(self.timestep)

        step = random.choice([-1, 1])
        self.multiple += step
        
        if self.isWrapOn:
            self.multiple %= 8
        else:
            if self.multiple < 0:
                self.multiple = 0
            elif self.multiple > 7:
                self.multiple = 7
        
    def stop(self):
        self.running = False
        self.__shifter.shiftByte(0b00000000)
