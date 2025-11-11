import time
import threading

# Shared shift register output across all motors
shifter_outputs = 0
shifter_lock = threading.Lock()

class Stepper:
    SEQUENCE = [0b0001,0b0011,0b0010,0b0110,0b0100,0b1100,0b1000,0b1001]

    def __init__(self, shifter, bit_offset=0, delay_us=1200):
        """
        shifter: your existing Shifter instance
        bit_offset: starting bit in shift register (0,4,8,...)
        delay_us: microseconds between steps
        """
        self.shifter = shifter
        self.bit_offset = bit_offset
        self.delay_us = delay_us
        self.step_index = 0
        self.angle = 0
        self.running = False
        self.thread = None

    def __sgn(self, x):
        return 0 if x == 0 else int(abs(x)/x)

    def __step(self, direction):
        global shifter_outputs
        self.step_index = (self.step_index + direction) % 8
        pattern = Stepper.SEQUENCE[self.step_index] << self.bit_offset
        mask = 0b1111 << self.bit_offset

        # Thread-safe update of shared shift register
        with shifter_lock:
            shifter_outputs &= ~mask
            shifter_outputs |= pattern
            self.shifter.shiftByte(shifter_outputs)

        self.angle = (self.angle + direction / (4096/360)) % 360

    def __rotate(self, steps):
        self.running = True
        direction = self.__sgn(steps)
        for _ in range(abs(steps)):
            if not self.running:
                break
            self.__step(direction)
            time.sleep(self.delay_us / 1e6)
        self.running = False

    def rotate(self, steps):
        """Start rotation in a thread and return the thread handle."""
        if self.thread and self.thread.is_alive():
            return
        self.thread = threading.Thread(target=self.__rotate, args=(steps,))
        self.thread.start()
        return self.thread

    def goAngle(self, target_angle):
        """Rotate to absolute angle using shortest path."""
        delta = (target_angle - self.angle) % 360
        if delta > 180:
            delta -= 360
        return self.rotate(int(delta * (4096/360)))

    def stop(self):
        self.running = False
