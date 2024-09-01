#!/usr/bin/env python3
##
# @file time_stepping.py
#
# @brief Provide implementation of time stepping for autonomous driving.
# autonomous driving.
#
# @section author_doxygen_example Author(s)
# - Created by Tran Viet Thanh on 03/08/2024.
#
# Copyright (c) 2024 System Engineering Laboratory.  All rights reserved.

# Standard library
import numpy as np


class TimeStepping:
    t_start = 0

    dt = 0.05

    # ==================================================================
    # PUBLIC METHODS
    # ==================================================================
    def __init__(self, model, trajectory, controller, observer, t_max=60):
        """! Constructor
        @param model<instance>: The vehicle model
        @param trajectory<instance>: The trajectory
        @param controller<instance>: The controller
        @param observer<instance>: The observer
        """
        self.t_out = []

        self.x_out = []

        self.y_out = []

        self.u_out = []

        self.dudt_out = []

        self.ddudt_out = []

        self.model = model

        self.trajectory = trajectory

        self.controller = controller

        self.observer = observer

        self._goal_tolerance = 0.1

        self._t_max = t_max

    def run(self, q0):
        """! Run the time stepping
        @param q0<list>: The initial state
        """
        self.t_out = np.linspace(
            TimeStepping.t_start,
            self._t_max,
            int((1 / TimeStepping.dt) * self._t_max),
        )

        nt = self.t_out.shape[-1]

        self.x_out = np.zeros([self.model.nx, nt])

        self.y_out = np.zeros([self.model.nx, nt])

        # Intialize time stepping
        self.x_out[:, 0] = q0

        self.y_out[:, 0] = q0

        self.u_out = np.zeros([self.model.nu, nt])

        self.dudt_out = np.zeros([self.model.nu, nt])

        self.ddudt_out = np.zeros([self.model.nu, nt])

        self.controller.initialize()

        for index in range(nt):
            y_m = self.y_out[:, index]

            u_m = self.u_out[:, index]

            status, u_m = self.controller.execute(
                y_m, u_m, index)

            if not status:
                self.u_out[:, index] = np.zeros([self.model.nu, 1])

            self.dudt_out[:, index] = (
                u_m - self.u_out[:, index]) / TimeStepping.dt

            x_m = self.model.function(
                self.x_out[:, index], self.u_out[:, index], TimeStepping.dt
            )

            y_m = x_m

            if index < nt - 1:
                self.x_out[:, index + 1] = x_m

                self.y_out[:, index + 1] = y_m

                self.u_out[:, index + 1] = u_m

            # For DWA, it is nessary to comment out the following lines athe moment
            # if self._is_goal(x_m, self.trajectory.x[-1]):
            #     break

        self.u_out = self.u_out[:, : index + 1]

        self.x_out = self.x_out[:, : index + 1]

        self.y_out = self.y_out[:, : index + 1]

    # ==================================================================
    # PRIVATE METHODS
    # ==================================================================
    def _is_goal(self, state, goal):
        """! Check if the vehicle reaches the goal
        @param state<list>: The state of the vehicle
        @param goal<list>: The goal
        @return<bool>: The result
        """
        distance = np.linalg.norm(state - goal)

        return distance < self._goal_tolerance
