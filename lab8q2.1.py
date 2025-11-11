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
   
    Motor instantiation sequence is inverted from the shift register outputs.
    For example, in the case of 2 motors, the 2nd motor must be connected
    with the first set of shift register outputs (Qa-Qd), and the 1st motor
    with the second set of outputs (Qe-Qh). This is because the MSB of
    the register is associated with Qa, and the LSB with Qh.
 
    An instance attribute (shifter_bit_start) tracks the bit position
    in the shift register where the 4 control bits for each motor begin.
    """

    # Class attributes:
    num_steppers = 0      # track number of Steppers instantiated
    shifter_outputs = 0   # track shift register outputs for all motors
    seq = [0b0001,0b0011,0b0010,0b0110,0b0100,0b1100,0b1000,0b1001] # CCW sequence
    delay = 1200          # delay between motor steps [us]
    steps_per_degree = 4096/360    # 4096 steps/rev * 1/360 rev/deg

    def __init__(self, shifter, lock):
        self.s = shifter
        self.angle = multiprocessing.Value('d', 0.0)  # shared across processes
        self.step_state = 0
        self.shifter_bit_start = 4*Stepper.num_steppers
        self.lock = lock
        Stepper.num_steppers += 1

    # Signum function:
    def __sgn(self, x):
        if x == 0: 
            return 0
        else: 
            return int(abs(x)/x)

    # Move a single +/-1 step in the motor sequence:
    def __step(self, dir):
        self.step_state += dir
        self.step_state %= 8

        # Clear only this motor's 4 bits:
        Stepper.shifter_outputs &= ~(0b1111 << self.shifter_bit_start)
        # Set the new 4-bit pattern for this motor:
        Stepper.shifter_outputs |= (Stepper.seq[self.step_state] << self.shifter_bit_start)
        # Send updated output to shift register:
        self.s.shiftByte(Stepper.shifter_outputs)

        # Update angle safely
        with self.angle.get_lock():
            self.angle.value += dir / Stepper.steps_per_degree
            self.angle.value %= 360

    # Move relative angle from current position (for rotate method):
    def __rotate(self, delta):
        numSteps = int(Stepper.steps_per_degree * abs(delta))
        dir = self.__sgn(delta)
        for _ in range(numSteps):
            self.__step(dir)
            time.sleep(Stepper.delay / 1e6)

    # Move relative angle in a separate process
    def rotate(self, delta):
        time.sleep(0.1)
        p = multiprocessing.Process(target=self.__rotate, args=(delta,))
        p.start()

    # Move to an absolute angle taking the shortest possible path:
    def goAngle(self, target_angle):
        """
        Rotate motor to the absolute angle `target_angle` using the shortest path.
        """
        def _rotate_to_target(shared_angle, delta):
            numSteps = int(Stepper.steps_per_degree * abs(delta))
            dir = 1 if delta > 0 else -1
            for _ in range(numSteps):
                self.__step(dir)
                time.sleep(Stepper.delay / 1e6)

        with self.angle.get_lock():
            current_angle = self.angle.value

        # Shortest path delta
        delta = (target_angle - current_angle) % 360
        if delta > 180:
            delta -= 360

        p = multiprocessing.Process(target=_rotate_to_target, args=(self.angle, delta))
        p.start()

    # Set the motor zero point
    def zero(self):
        with self.angle.get_lock():
            self.angle.value = 0.0


# Example use:
if __name__ == '__main__':

    s = Shifter(data=16, latch=20, clock=21)   # set up Shifter

    # multiprocessing.Lock() still defined but not used
    lock = multiprocessing.Lock()

    # Instantiate 2 Steppers:
    m1 = Stepper(s, lock)
    m2 = Stepper(s, lock)

    # Zero the motors:
    m1.zero()
    m2.zero()
    m1.rotate(90)
    m1.rotate(-45)
    m2.rotate(-90)
    m2.rotate(45)
    m1.rotate(-135)
    m1.rotate(135)
    m1.rotate(0)

    # While the motors are running in their separate processes,
    # the main code can continue doing its thing: 
    try:
        while True:
            pass
    except:
        print('\nend')
