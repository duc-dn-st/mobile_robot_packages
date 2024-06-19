
# Standard library 
import math 

# External library 
import numpy as np


class DifferentialDrive: 
    # [x, y, theta]
    nx = 3

    # [v, w]
    nu = 2

    def __init__(self, wheel_base):
        self.wheel_base = wheel_base

        self.velocity_max = 1.0 # m/s

    def function(self, state, input, dt):
        v = input[0]

        w = input[1]

        dfdt = np.array([math.cos(state[2]) * v, math.sin(state[2]) * v, w])

        next_state = state + dfdt * dt

        return next_state