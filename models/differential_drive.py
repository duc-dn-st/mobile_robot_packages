
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

        self.velocity_max = 1.0

        self.angular_velocity_max = 1.0

    def function(self, state, input, dt):
        v = input[0]

        w = input[1]

        dfdt = np.array([math.cos(state[2]) * v, math.sin(state[2]) * v, w])

        next_state = state + dfdt * dt

        return next_state
    
    def get_state_space_matrices(self, state, input, dt):
        """! Get the state space matrices by Jacobian linearization
        @param state<list>: The state of the vehicle
        @param input<list>: The input of the vehicle
        @param dt<float>: The time step
        @return A, B<tuple>: The state space matrices
        """

        v = input[0]

        A = np.array([[1, 0, -math.sin(state[2]) * v * dt],
                      [0, 1, math.cos(state[2]) * v * dt],
                      [0, 0, 1]])

        B = np.array([[math.cos(state[2]) * dt, 0],
                      [math.sin(state[2]) * dt, 0],
                      [0, dt]])

        return A, B
