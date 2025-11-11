# stepper_class_shiftregister_multiprocessing.py
#
# Stepper class supporting simultaneous multi-motor operation
# via multiprocessing and shift registers (e.g., 74HC595).
#

import time
import multiprocessing
from shifter import Shifter   # uses your updated Shifter class

class Stepper:
    """
    Stepper motor controller that uses shared shift register outputs
    so multiple motors can run simultaneously.
    """

    # Class attributes
    num_steppers = 0
    shifter_outputs = 0
    seq = [0b0001,0b0011,0b0010,0b0110,0b0100,0b1100,0b1000,0b1001]  # 8-step half sequence
    delay = 1200   # microseconds between steps
    steps_per_degree = 4096 / 360  # 4096 steps/rev

    def __init__(self, shifter, lock):
        self.s = shifter
        self.angle = 0
        self.step_state = 0
        self.shifter_bit_start = 4 * Stepper.num_steppers
        self.lock = lock
        Stepper.num_steppers += 1

    # Sign function
    def __sgn(self, x):
        if x == 0:
            return 0
        else:
            return int(abs(x) / x)

    # Take a single step
    def __step(self, direction):
        self.step_state = (self.step_state + direction) % 8
        pattern = Stepper.seq[self.step_state] << self.shifter_bit_start

        # Clear only this motor's 4 bits, then set them
        mask = ~(0b1111 << self.shifter_bit_start)
        Stepper.shifter_outputs &= mask
        Stepper.shifter_outputs |= pattern

        # Send data to shift register
        self.s.shiftByte(Stepper.shifter_outputs)

        # Track angle
        self.angle = (self.angle + direction / Stepper.steps_per_degree) % 360

    # Rotate motor by relative angle
    def __rotate(self, delta):
        numSteps = int(Stepper.steps_per_degree * abs(delta))
        direction = self.__sgn(delta)

        for _ in range(numSteps):
            self.__step(direction)
            time.sleep(Stepper.delay / 1e6)

    # Public rotate method (non-blocking, runs in new process)
    def rotate(self, delta):
        p = multiprocessing.Process(target=self.__rotate, args=(delta,))
        p.start()
        return p  # return process handle to allow waiting later

    # Move to a specific absolute angle (shortest path)
    def goAngle(self, angle):
        angle %= 360
        delta = angle - self.angle
        if delta > 180:
            delta -= 360
        elif delta < -180:
            delta += 360
        return self.rotate(delta)

    # Zero the motor angle
    def zero(self):
        self.angle = 0


# Example main program
if __name__ == '__main__':
    import RPi.GPIO as GPIO

    s = Shifter(data=16, latch=20, clock=21)
    lock = multiprocessing.Lock()

    # Create two motors
    m1 = Stepper(s, lock)
    m2 = Stepper(s, lock)

    # Zero them
    m1.zero()
    m2.zero()

    # Start both motors simultaneously
    p1 = m1.rotate(180)
    p2 = m2.rotate(-180)

    # Wait for both to finish
    p1.join()
    p2.join()

    print("Both motors finished moving.")

    # Clean up GPIO
    GPIO.cleanup()
