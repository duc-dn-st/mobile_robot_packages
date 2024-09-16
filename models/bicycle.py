#!/usr/bin/env python3
##
# @file bicycle.py
#
# @brief Provide implementation of the bicycle model.
#
# @section author_doxygen_example Author(s)
# - Created by Tran Viet Thanh on 2024/09/16.
#
# Copyright (c) 2024 System Engineering Laboratory.  All rights reserved.

# Standard library
import math

# External library
import numpy as np


class Bicycle:
    """! Bicycle model

    The class provides implementation of the bicycle model.
    @note The model is used to simulate the vehicle motion.
    @variable nx: The number of states. The states are [x, y, theta, delta]
    @variable nu: The number of inputs. The inputs are [v, w]
    """
    nx = 4

    nu = 2

    def __init__(self, lengh_base):
        """! Constructor
        @param wheel_base<float>: The wheel base of the vehicle
        @note The wheel base is the distance between the front and rear axles.
        """
        self.lengh_base = lengh_base

        self.velocity_max = 1.0

    def function(self, state, input, dt):
        """! Compute the next state
        @param state<list>: The current state of the vehicle
        @param input<list>: The input of the vehicle
        @param dt<float>: The time step
        @return next_state<list>: The next state of the vehicle
        """
        v = input[0]

        w = input[1]

        dfdt = np.array([v * math.cos(state[2]),
                         v * math.sin(state[2]),
                         w,
                         0])

        next_state = state + dfdt * dt

        next_state[3] = math.atan2(w * self.lengh_base, v) if v != 0 else 0

        return next_state

    def calculate_front_alxe(self, state, input):
        """! Calculate the front axle
        @param state<list>: The state of the vehicle
        @return front_axle<list>: The front axle of the vehicle
        """
        v = input[0]

        w = input[1]

        delta = math.atan2(w * self.lengh_base, v) if v != 0 else 0

        v_front = v / math.cos(delta)

        return np.array([v_front, delta])
