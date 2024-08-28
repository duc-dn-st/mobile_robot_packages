#!/usr/bin/env python3
##
# @file plotter.py
#
# @brief Provide plotting functions for the system.
#
# @section author_doxygen_example Author(s)
# - Created by Tran Viet Thanh on 27/08/2024.
#
# Copyright (c) 2024 System Engineering Laboratory.  All rights reserved.

# Standard library
import csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


class Plotter:
    """! Plotter class for plotting data.

    This class provides functions for plotting data.
    """
    # =========================================================================
    # PUBLIC METHODS
    # =========================================================================

    def __init__(self, simulator, trajectory, environment):
        """! The constructor of the class.
        @param simulator The simulator object.
        @param trajectory The trajectory object.
        @param environment The environment object.
        """
        self._simulator = simulator

        self._trajectory = trajectory

        self._environment = environment

    def plot(self):
        """! Plot the data.
        """
        self._plot_positions()

        self._plot_velocity()

        self._plot_acceleration()

        plt.show()

    # =========================================================================
    # PRIVATE METHODS
    # =========================================================================
    def _plot_positions(self):
        """! Plot the positions.
        """
        _, ax = plt.subplots(1, 1)

        ax.set_box_aspect(1)

        ax.plot(self._simulator.x_out[0, :],
                self._simulator.x_out[1, :], "r",
                label="Tracking performance")

        ax.plot(
            [path[0] for path in self._trajectory.x],
            [path[1] for path in self._trajectory.x],
            "--b", label="Trajectory",
        )

        self._plot_obstacles(ax)

        ax.legend()

        ax.set_xlabel("X [m]")

        ax.set_ylabel("Y [m]")

        plt.tight_layout()

    def _plot_obstacles(self, ax):
        """! Plot the obstacles.
        @param ax The axis object.
        """
        resolution = 0.125

        with open(self._environment, 'r') as f:
            reader = csv.reader(f, delimiter=" ")

            lines = list(reader)

        lines = list(map(lambda line: list(map(int, line)), lines))

        array = np.array(lines)

        for row in range(array.shape[0]):
            for column in range(array.shape[1]):
                if array[row, column] != 1:
                    continue

                x = column * resolution

                y = (array.shape[0] - row - 1) * resolution

                rect = Rectangle((x, y), resolution,
                                 resolution, facecolor='black')

                ax.add_patch(rect)

    def _plot_velocity(self):
        """! Plot the velocity.
        """
        _, (linear_ax, angular_ax) = plt.subplots(1, 2)

        number_of_input = len(self._simulator.u_out[0, :])

        linear_ax.plot(self._simulator.t_out[:number_of_input],
                       self._simulator.u_out[0, :])

        angular_ax.plot(self._simulator.t_out[:number_of_input],
                        self._simulator.u_out[1, :])

        linear_ax.set_xlabel("Time [s]")

        linear_ax.set_ylabel("Velocity [m/s]")

        angular_ax.set_xlabel("Time [s]")

        angular_ax.set_ylabel("Angular Velocity [rad/s]")

        plt.tight_layout()

    def _plot_acceleration(self):
        """! Plot the acceleration.
        """
        _, (linear_ax, angular_ax) = plt.subplots(1, 2)

        linear_ax.plot(self._simulator.t_out, self._simulator.dudt_out[0, :])

        angular_ax.plot(self._simulator.t_out, self._simulator.dudt_out[1, :])

        linear_ax.set_xlabel("Time [s]")

        linear_ax.set_ylabel("Acceleration [m/s^2]")

        angular_ax.set_xlabel("Time [s]")

        angular_ax.set_ylabel("Angular Acceleration [rad/s^2]")

        plt.tight_layout()
