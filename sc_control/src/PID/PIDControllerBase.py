import numpy as np
import rospy

class PIDControllerBase():
    def __init__(self, _p, _i, _d, _sat):
        self.Kp = _p
        self.Kd = _i
        self.Ki = _d
        self.sat = _sat

        self.integral = 0 
        self.prev_err = 0
        self.prev_t = -1.0
    
    def regulate(self, _err, _t):
        derr_dt = 0.0
        dt = _t - self.prev_t
        if self.prev_t > 0.0 and dt > 0.0:
            derr_dt = (_err - self.prev_err) / dt
            self.integral += 0.5 * (_err + self.prev_err) * dt

        u = self.Kp * _err + self.Kd * derr_dt + self.Ki * self.integral

        self.prev_err = _err
        self.prev_t = _t

        if (np.linalg.norm(u) > self.sat):
            # controller is in saturation: limit output, reset integral
            u = self.sat * u / np.linalg.norm(u)
            self.integral = 0.0

        return u
