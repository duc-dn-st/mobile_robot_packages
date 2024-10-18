#!/usr/bin/env python3
##
# @file run.py
#
# @brief Provide plotting functions for the system.
#
# @section author_doxygen_example Author(s)
# - Created by Tran Viet Thanh on 27/08/2024.
#
# Copyright (c) 2024 System Engineering Laboratory.  All rights reserved.

# Standard library
import os
import sys
import numpy as np
import math

# Internal library
sys.path.append('..')
from visualizers.plotter import Plotter
from simulators.time_stepping import TimeStepping
from models.bicycle import Bicycle
from controllers.feedforward import FeedForward
from trajectory_generators.simple_generator import SimpleGenerator


if __name__ == "__main__":
    wheel_base = 1.0

    model = Bicycle(wheel_base)

    trajectory = SimpleGenerator(model)

    trajectory.generate("bicycle_l_shape_ep0.25_1s.csv", nx=3, nu=2,
                        is_derivative=False)

    current_folder = os.path.dirname(os.path.abspath(__file__))

    map_folder = os.path.abspath(os.path.join(
        current_folder, 'perception', 'maps'))

    environment = ''

    controller = FeedForward(model, trajectory)

    simulator = TimeStepping(model, trajectory, controller, None, t_max=trajectory.t[-1]+1)

    plotter = Plotter(simulator, trajectory, environment)

    simulator._dt = 1.0

    simulator.run([0.0, 0.0, 0.0])

    plotter.plot()
