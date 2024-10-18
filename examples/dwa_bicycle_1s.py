#!/usr/bin/env python3
##
# @file dwa_with_l_shape.py
#
# @brief Provide example for DWA tracking L-shape trajectory.
#
# @section author_doxygen_example Author(s)
# - Created by Tran Viet Thanh on 27/08/2024.

# Standard library
import sys
import math

# Internal library
sys.path.append('..')
from visualizers.plotter import Plotter  # noqa
from controllers.dwa_bicycle import DynamicWindowApproach  # noqa
from simulators.time_stepping import TimeStepping  # noqa
from models.bicycle import Bicycle  # noqa
from trajectory_generators.simple_generator import SimpleGenerator  # noqa


if __name__ == "__main__":
    wheel_base = 1.0

    model = Bicycle(wheel_base)

    trajectory = SimpleGenerator(model)

    trajectory.generate("bicycle_l_shape_ep0.4_1s.csv", nx=3, nu=2,
                        is_derivative=False)

    controller = DynamicWindowApproach(model, trajectory)

    simulator = TimeStepping(model, trajectory, controller, None, t_max=180)

    plotter = Plotter(simulator, trajectory)

    simulator._dt = 1.0

    simulator.run([0.0, 0.0, 0.0])

    plotter.plot()
