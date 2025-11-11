# stepper_class_shiftregister_threading.py
import time
import threading
from shifter import Shifter   # keep your existing Shifter implementation

class Stepper:
    # Class attributes:
    num_steppers = 0
    shifter_outputs = 0   # combined outputs for all shift registers
    seq = [0b0001,0b0011,0b0010,0b0110,0b0100,0b1100,0b1000,0b1001]  # CCW
    delay = 1200          # delay between motor steps [us]
    steps_per_degree = 4096.0 / 360.0

    # Lock protecting access to shifter_outputs and actual shift register writes
    shifter_lock = threading.Lock()

    def __init__(self, shifter):
        """
        Note: using threads rather than multiprocessing. Threads share memory,
        so self.angle is a normal float protected by self.angle_lock.
        """
        self.s = shifter
        self.angle = 0.0
        self.angle_lock = threading.Lock()
        self.step_state = 0
        self.shifter_bit_start = 4 * Stepper.num_steppers
        Stepper.num_steppers += 1

    # Sign function
    def __sgn(self, x):
        if x == 0:
            return 0
        return 1 if x > 0 else -1

    # Low-level single step (dir = +1 or -1)
    def __step(self, dir):
        # Update step state
        self.step_state = (self.step_state + dir) % len(Stepper.seq)

        # Update the shared shifter_outputs in a thread-safe manner
        mask = 0b1111 << self.shifter_bit_start

        with Stepper.shifter_lock:
            # Clear this motor's 4 bits
            Stepper.shifter_outputs &= ~mask
            # Set the new bits for this motor
            Stepper.shifter_outputs |= (Stepper.seq[self.step_state] << self.shifter_bit_start)
            # Write out to shift register
            self.s.shiftByte(Stepper.shifter_outputs)

        # Update this motor's angle (shared memory so other threads see it)
        with self.angle_lock:
            self.angle = (self.angle + dir / Stepper.steps_per_degree) % 360.0

    # Internal rotation routine run in a thread
    def _rotate_thread(self, delta):
        num_steps = int(abs(delta) * Stepper.steps_per_degree + 0.5)
        direction = self.__sgn(delta)
        for _ in range(num_steps):
            self.__step(direction)
            # sleep using microsecond delay as in original
            time.sleep(Stepper.delay / 1e6)

    # Public rotate (relative, non-blocking)
    def rotate(self, delta):
        # small debounce
        time.sleep(0.01)
        t = threading.Thread(target=self._rotate_thread, args=(delta,), daemon=True)
        t.start()
        return t  # return thread if caller wants to join

    # Move to an absolute angle by shortest path (non-blocking)
    def goAngle(self, target_angle):
        # Normalize target to [0,360)
        target = target_angle % 360.0
        with self.angle_lock:
            current = self.angle
        # Compute shortest delta in range [-180, +180)
        delta = ((target - current + 540.0) % 360.0) - 180.0
        return self.rotate(delta)

    # Set motor zero
    def zero(self):
        with self.angle_lock:
            self.angle = 0.0
        # Also clear the motor outputs for this motor (optional)
        mask = 0b1111 << self.shifter_bit_start
        with Stepper.shifter_lock:
            Stepper.shifter_outputs &= ~mask
            self.s.shiftByte(Stepper.shifter_outputs)


# Example usage:
if __name__ == '__main__':
    s = Shifter(data=16, latch=20, clock=21)
    m1 = Stepper(s)
    m2 = Stepper(s)

    m1.zero()
    m2.zero()

    # Example sequence — calls are non-blocking and should run concurrently
    m1.goAngle(90)
    m1.goAngle(-45)

    m2.goAngle(-90)
    m2.goAngle(45)

    m1.goAngle(-135)
    m1.goAngle(135)
    m1.goAngle(0)

    try:
        # keep main thread alive while motors run
        while True:
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\nEnd")
