# stepper_class_shiftregister_multiprocessing.py
#
# Stepper class with simultaneous absolute moves
import time
import multiprocessing
from shifter import Shifter

class Stepper:
    num_steppers = 0
    shifter_outputs = 0
    seq = [0b0001,0b0011,0b0010,0b0110,0b0100,0b1100,0b1000,0b1001]
    delay = 1200
    steps_per_degree = 4096/360

    def __init__(self, shifter, lock):
        self.s = shifter
        self.angle = multiprocessing.Value('d', 0.0)
        self.step_state = 0
        self.shifter_bit_start = 4*Stepper.num_steppers
        self.lock = lock
        Stepper.num_steppers += 1

    def __sgn(self, x):
        return 0 if x == 0 else int(abs(x)/x)

    def __step(self, dir):
        self.step_state = (self.step_state + dir) % 8
        Stepper.shifter_outputs &= ~(0b1111 << self.shifter_bit_start)
        Stepper.shifter_outputs |= (Stepper.seq[self.step_state] << self.shifter_bit_start)
        self.s.shiftByte(Stepper.shifter_outputs)
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
        return p  # return process so main can join

    def goAngle(self, target_angle):
        """Rotate motor to an absolute angle using the shortest path."""
        with self.angle.get_lock():
            current_angle = self.angle.value
        delta = (target_angle - current_angle) % 360
        if delta > 180:
            delta -= 360
        return self.rotate(delta)

    def zero(self):
        with self.angle.get_lock():
            self.angle.value = 0.0


# Example usage
if __name__ == '__main__':
    s = Shifter(data=16, latch=20, clock=21)
    lock = multiprocessing.Lock()

    m1 = Stepper(s, lock)
    m2 = Stepper(s, lock)

    # Zero the motors
    m1.zero()
    m2.zero()

    # Sequence of moves
    sequence = [
        (m1, 90),
        (m1, -45),
        (m2, -90),
        (m2, 45),
        (m1, -135),
        (m1, 135),
        (m1, 0)
    ]

    # Run moves
    for motor, angle in sequence:
        p = motor.goAngle(angle)
        p.join()  # wait until move is finished

    print("All moves complete.")

