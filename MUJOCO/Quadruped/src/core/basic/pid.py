import numpy as np

class SimplePID:
    def __init__(self, kp, ki, kd, dt, wrap=False):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.wrap = wrap
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, target, current):
        if self.wrap:
            error = (target - current + np.pi) % (2 * np.pi) - np.pi
        else:
            error = target - current

        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0