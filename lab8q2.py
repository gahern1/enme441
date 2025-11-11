import RPi.GPIO as GPIO
import time
import random

GPIO.setmode(GPIO.BCM)

class Shifter:
    def __init__(self, data, latch, clock):
        self.dataPin = data
        self.latchPin = latch
        self.clockPin = clock

        GPIO.setup(self.dataPin, GPIO.OUT)
        GPIO.setup(self.latchPin, GPIO.OUT, initial=0)
        GPIO.setup(self.clockPin, GPIO.OUT, initial=0)

    def __ping(self, pin):
        GPIO.output(pin, 1)
        time.sleep(0)
        GPIO.output(pin, 0)

    def shiftByte(self, value):
        for i in range(8):
            GPIO.output(self.dataPin, value & (1 << i))
            self.__ping(self.clockPin)
        self.__ping(self.latchPin)


class Bug:
    def __init__(self, shifter, timestep=0.1, multiple=3, isWrapOn=False):
        self.timestep = timestep
        self.multiple = multiple
        self.isWrapOn = isWrapOn
        self.shifter = shifter
        self.running = False

    def move_once(self):
        if not self.running:
            return

        pattern = 1 << self.multiple
        self.shifter.shiftByte(pattern)
        time.sleep(self.timestep)

        step = random.choice([-1, 1])
        self.multiple += step

        if self.isWrapOn:
            self.multiple %= 8
        else:
            self.multiple = max(0, min(7, self.multiple))

    def stop(self):
        self.running = False
        self.shifter.shiftByte(0b00000000)
