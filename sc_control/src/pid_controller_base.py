import numpy as np
import rospy

class PIDControllerBase():
    def __init__(self, *args):
        self._Kp = 0
        self._Kd = 0
        self._Ki = 0
