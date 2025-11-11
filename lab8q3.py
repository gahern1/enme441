# stepper_class_shiftregister_multiprocessing.py
#
# Stepper class for simultaneous multi-motor control with shortest-path absolute rotation.

import time
import multiprocessing
from shifter import Shifter   # use your existing Shifter class

class Stepper:
    """
    Stepper motor class supporting multiple motors on shift registers.
    Uses multiprocessing.Value for cross-process shared angle tracking.
    """

    # Class attributes:
    num_steppers = 0
    shifter_outputs = 0
    seq = [0b0001,0b0011,0b0010,0b0110,0b0100,0b1100,0b1000,0b1001]  # CCW sequence
    delay = 1200  # µs between steps
    steps_per_degree = 4096 / 360  # 4096 steps per revolution

    def __init__(self, shifter, lock=None):
        self.s = shifter
        self.angle = multiprocessing.Value('d', 0.0)  # shared angle across processes
        self.step_state = 0
        self.shifter_bit_start = 4 * Stepper.num_steppers
        self.lock = lock  # optional, unused here
        Stepper.num_steppers += 1

    # Internal sign function
    def __sgn(self, x):
        return 0 if x == 0 else int(abs(x)/x)

    # Step one step in the given direction (+1/-1)
    def __step(self, dir):
        self.step_state = (self.step_state + dir) % 8

        # Update only this motor's 4 bits
        Stepper.shifter_outputs &= ~(0b1111 << self.shifter_bit_start)
        Stepper.shifter_outputs |= (Stepper.seq[self.step_state] << self.shifter_bit_start)
        self.s.shiftByte(Stepper.shifter_outputs)

        # Update shared angle value
        self.angle.value = (self.angle.value + dir / Stepper.steps_per_degree) % 360

    # Internal rotate method (used in child process)
    def __rotate(self, delta):
        num_steps = int(Stepper.steps_per_degree * abs(delta))
        dir = self.__sgn(delta)
        for _ in range(num_steps):
            self.__step(dir)
            time.sleep(Stepper.delay / 1e6)

    # Public rotate: spawns a process to rotate relative delta
    def rotate(self, delta):
        time.sleep(0.05)  # small delay to prevent race conditions on start
        p = multiprocessing.Process(target=self.__rotate, args=(delta,))
        p.start()
        return p  # return the process handle so caller can join()

    # Go to absolute angle a using shortest-path
    def goAngle(self, a):
        delta = a - self.angle.value
        if delta > 180:
            delta -= 360
        elif delta < -180:
            delta += 360
        return self.rotate(delta)

    # Zero the motor
    def zero(self):
        self.angle.value = 0.0


# --- Example usage ---
if __name__ == '__main__':
    import RPi.GPIO as GPIO

    GPIO.setwarnings(False)

    # Initialize shift register
    s = Shifter(data=16, latch=20, clock=21)
    lock = multiprocessing.Lock()  # optional, not needed here

    # Create two motors
    m1 = Stepper(s, lock)
    m2 = Stepper(s, lock)

    # Zero motors
    m1.zero()
    m2.zero()

    # Start simultaneous rotations and collect processes
    processes = []
    processes.append(m1.goAngle(90))
    processes.append(m2.goAngle(-90))
    processes.append(m1.goAngle(-45))
    processes.append(m2.goAngle(45))
    processes.append(m1.goAngle(-135))
    processes.append(m1.goAngle(135))
    processes.append(m1.goAngle(0))

    # Wait for all motor processes to finish
    for p in processes:
        if p is not None:
            p.join()

    # Turn off all outputs
    s.shiftByte(0b00000000)
    GPIO.cleanup()
    print("All motors finished")

