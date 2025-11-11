# stepper_class_shiftregister_multiprocessing.py
#
# Stepper class
#
# Because only one motor action is allowed at a time, multithreading could be
# used instead of multiprocessing. However, the GIL makes the motor process run 
# too slowly on the Pi Zero, so multiprocessing is needed.

import time
import multiprocessing
from shifter import Shifter   # our custom Shifter class

class Stepper:
    """
    Supports operation of an arbitrary number of stepper motors using
    one or more shift registers.
  
    A class attribute (shifter_outputs) keeps track of all
    shift register output values for all motors.  In addition to
    simplifying sequential control of multiple motors, this schema
    also makes simultaneous operation of multiple motors possible.
    """

    # Class attributes:
    num_steppers = 0
    shifter_outputs = 0
    seq = [0b0001,0b0011,0b0010,0b0110,0b0100,0b1100,0b1000,0b1001] # CCW sequence
    delay = 1200          # delay between motor steps [us]
    steps_per_degree = 4096/360

    def __init__(self, shifter, lock):
        self.s = shifter
        self.angle = multiprocessing.Value('d', 0.0)  # shared across processes
        self.step_state = 0
        self.shifter_bit_start = 4*Stepper.num_steppers
        self.lock = lock
        Stepper.num_steppers += 1

    def __sgn(self, x):
        return 0 if x == 0 else int(abs(x)/x)

    def __step(self, dir):
        self.step_state = (self.step_state + dir) % 8

        # Update shift register bits for this motor
        Stepper.shifter_outputs &= ~(0b1111 << self.shifter_bit_start)
        Stepper.shifter_outputs |= (Stepper.seq[self.step_state] << self.shifter_bit_start)
        self.s.shiftByte(Stepper.shifter_outputs)

        # Update shared angle safely
        with self.angle.get_lock():
            self.angle.value += dir / Stepper.steps_per_degree
            self.angle.value %= 360

    def __rotate(self, delta):
        numSteps = int(Stepper.steps_per_degree * abs(delta))
        dir = self.__sgn(delta)
        for _ in range(numSteps):
            self.__step(dir)
            time.sleep(Stepper.delay / 1e6)

    def rotate(self, delta):
        """Rotate motor by a relative delta in a separate process."""
        time.sleep(0.1)
        p = multiprocessing.Process(target=self.__rotate, args=(delta,))
        p.start()
        return p  # return process so main can join it

    def goAngle(self, target_angle):
        """Rotate motor to an absolute angle using shortest path."""
        with self.angle.get_lock():
            current_angle = self.angle.value

        # Compute shortest path
        delta = (target_angle - current_angle) % 360
        if delta > 180:
            delta -= 360

        return self.rotate(delta)

    def zero(self):
        with self.angle.get_lock():
            self.angle.value = 0.0


# Example usage:
if __name__ == '__main__':
    s = Shifter(data=16, latch=20, clock=21)
    lock = multiprocessing.Lock()

    # Instantiate 2 steppers
    m1 = Stepper(s, lock)
    m2 = Stepper(s, lock)

    # Zero the motors
    m1.zero()
    m2.zero()

    # Example sequence of absolute positions
    sequence = [
        (90, -45),
        (-90, 45),
        (-135, 135),
        (0, 0)
    ]

    for angle1, angle2 in sequence:
        # Move both motors simultaneously
        p1 = m1.goAngle(angle1)
        p2 = m2.goAngle(angle2)
        # Wait for both to finish
        p1.join()
        p2.join()
        print(f"Motors reached angles: M1={angle1}, M2={angle2}")

    print("All moves complete.")

