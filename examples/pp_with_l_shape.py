#!/usr/bin/env python3
##
# @file pp_with_l_shape.py
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

# Internal library
sys.path.append('..')
from visualizers.plotter import Plotter  # noqa
from simulators.time_stepping import TimeStepping  # noqa
from models.differential_drive import DifferentialDrive  # noqa
from controllers.purepursuit.purepursuit import PurePursuit  # noqa
from trajectory_generators.simple_generator import SimpleGenerator  # noqa


if __name__ == "__main__":
    wheel_base = 0.53

    model = DifferentialDrive(wheel_base)

    trajectory = SimpleGenerator(model)

    trajectory.generate("global_trajectory.csv", nx=3, nu=2,
                        is_derivative=True)

    current_folder = os.path.dirname(os.path.abspath(__file__))

    map_folder = os.path.abspath(os.path.join(
        current_folder, '..', 'perception', 'maps'))

    environment = ''

    controller = PurePursuit(model, trajectory)

    simulator = TimeStepping(model, trajectory, controller, None, t_max=120)

    plotter = Plotter(simulator, trajectory, environment)

    simulator.run(0.0)

    plotter.plot()
