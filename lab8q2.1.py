# stepper_class_shiftregister_multiprocessing_v2.py
from RPi import GPIO
import time
import multiprocessing
from shifter import Shifter

class Stepper:
    num_steppers = 0
    shifter_outputs = 0
    seq = [0b0001,0b0011,0b0010,0b0110,0b0100,0b1100,0b1000,0b1001]
    delay = 1200
    steps_per_degree = 4096 / 360

    def __init__(self, shifter, lock):
        self.s = shifter
        self.angle = multiprocessing.Value('d', 0.0)   # shared value for angle
        self.step_state = 0
        self.shifter_bit_start = 4 * Stepper.num_steppers
        self.lock = lock
        Stepper.num_steppers += 1

    def __sgn(self, x):
        if x == 0:
            return 0
        return int(abs(x)/x)

    def __step(self, dir):
        self.step_state = (self.step_state + dir) % 8

        # isolate 4 bits for this motor
        mask = 0b1111 << self.shifter_bit_start
        Stepper.shifter_outputs &= ~mask
        Stepper.shifter_outputs |= (Stepper.seq[self.step_state] << self.shifter_bit_start)

        self.s.shiftByte(Stepper.shifter_outputs)

        with self.angle.get_lock():
            self.angle.value = (self.angle.value + dir / Stepper.steps_per_degree) % 360

    def __rotate(self, delta):
        self.lock.acquire()
        numSteps = int(Stepper.steps_per_degree * abs(delta))
        direction = self.__sgn(delta)
        for _ in range(numSteps):
            self.__step(direction)
            time.sleep(Stepper.delay / 1e6)
        self.lock.release()

    def rotate(self, delta):
        time.sleep(0.1)
        p = multiprocessing.Process(target=self.__rotate, args=(delta,))
        p.start()

    def goAngle(self, target_angle):
        target_angle = target_angle % 360
        with self.angle.get_lock():
            current = self.angle.value
        delta = (target_angle - current + 540) % 360 - 180
        p = multiprocessing.Process(target=self.__rotate, args=(delta,))
        p.start()

    def zero(self):
        with self.angle.get_lock():
            self.angle.value = 0.0


if __name__ == '__main__':
    s = Shifter(data=16, latch=20, clock=21)
    lock = multiprocessing.Lock()
    m1 = Stepper(s, lock)
    m2 = Stepper(s, lock)

    m1.zero()
    m2.zero()

    # Demo sequence
    m1.goAngle(90)
    m1.goAngle(-45)

    m2.goAngle(-90)
    m2.goAngle(45)

    m1.goAngle(-135)
    m1.goAngle(135)
    m1.goAngle(0)

    try:
        while True:
            pass
    except KeyboardInterrupt:
        print('\nend')
