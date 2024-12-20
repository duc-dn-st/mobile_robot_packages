#!/usr/bin/env python3
##
# @file vfh_purpursuit.py
#
# @brief Provide implementation of VFH Pure Pursuit for controllers.
#
# @section author_doxygen_example Author(s)
# - Created by Tran Viet Thanh on 27/08/2024.
#
# Copyright (c) 2024 System Engineering Laboratory.  All rights reserved.

# Standard library
import numpy as np

# Internal library
from perception.vfh.vector_field_histogram import VectorFieldHistogram


class VFHPurePursuit:
    """! VFH Pure Pursuit

    The class provides implementation of VFH Pure Pursuit for controllers.
    """
    lookahead_distance = 0.3

    lookahead_gain = 0.0

    k = 0.6
    # ==================================================================================================
    # PUBLIC METHODS
    # ==================================================================================================

    def __init__(self, model, trajectory, environment):
        """! Constructor
        @param model<instance>: The vehicle model
        @param trajectory<instance>: The trajectory
        @param environment<str>: The environment
        @note The trajectory is a set of waypoints.
        """
        self.model = model

        self.trajectory = trajectory

        self.old_nearest_point_index = None

        self._v = 0.0

        self._max_acceleration = 0.04

        self._w = 0.0

        self._vfh = VectorFieldHistogram(environment)

        self._debug = True

        self._debug_info = []

    def initialize(self):
        """! Initialize the controller
        @note The method is used to initialize the controller.
        """
        pass

    def execute(self, state, input, previous_index):
        """! Execute the controller
        @param state<list>: The state of the vehicle
        @param input<list>: The input of the vehicle
        @param previous_index<int>: The previous index
        @return<tuple>: The status and control
        """
        status = True

        index, lookahead_distance = self._search_target_index(
            state, input, VFHPurePursuit.lookahead_distance)

        if previous_index >= index:
            index = previous_index

        if index < len(self.trajectory.x):
            trajectory_x = self.trajectory.x[index, 0]

            trajectory_y = self.trajectory.x[index, 1]

        else:
            trajectory_x = self.trajectory.x[-1, 0]

            trajectory_y = self.trajectory.x[-1, 1]

            index = len(self.trajectory.x) - 1

        vfh_angle = self._retrieve_vfh_angle(
            state, [trajectory_x, trajectory_y])

        alpha = (
            vfh_angle
            - state[2]
        )

        a = self._apply_proportional_control(
            VFHPurePursuit.k, self.trajectory.u[0, index], self._v
        )

        a = self._max_acceleration if a > self._max_acceleration else a

        self._v = self._v + a * self.trajectory.sampling_time

        w = self._v * 2.0 * alpha / lookahead_distance

        # dwdt = min(
        #     max((w - self._w) / self.trajectory.sampling_time, -0.1),
        #     0.1)

        # w = self._w + dwdt * self.trajectory.sampling_time

        # self._w = w

        return status, [self._v, w]

    # ==================================================================================================
    # STATIC METHODS
    # ==================================================================================================
    @staticmethod
    def _apply_proportional_control(k_p, target, current):
        """! Apply proportional control
        @param k_p<float>: The proportional gain
        @param target<list>: The target
        @param current<list>: The current
        @return<float>: The acceleration
        """
        a = k_p * (target - current)

        return a

    @staticmethod
    def _calculate_distance(reference_x, current_x):
        """! Calculate the distance
        @param reference_x<list>: The reference x
        @param current_x<list>: The current x
        @return<float>: The distance
        """
        distance = current_x - reference_x

        x = distance[:, 0] if distance.ndim == 2 else distance[0]

        y = distance[:, 1] if distance.ndim == 2 else distance[1]

        return np.hypot(x, y)

    # ==================================================================================================
    # PRIVATE METHODS
    # ==================================================================================================
    def _retrieve_vfh_angle(self, state, target):
        """! Retrieve the VFH angle
        @param state<list>: The state of the vehicle
        @param target<list>: The target
        @return<float>: The angle
        """
        self._vfh.set_robot_location(state[:2])

        angle = self._vfh.get_best_angle(target)

        if self._debug:
            self._debug_info.append(
                (target[0], target[1], state[0], state[1], angle))

        return angle

    def _search_target_index(self, state, input, lookahead_distance):
        """! Search the target index
        @param state<list>: The state of the vehicle
        @param input<list>: The input of the vehicle
        @param lookahead_distance<float>: The lookahead distance
        @return<tuple>: The index and lookahead distance
        """
        if self.old_nearest_point_index is None:
            all_distance = self._calculate_distance(self.trajectory.x, state)

            index = np.argmin(all_distance)

        else:
            index = self.old_nearest_point_index

            this_distance = self._calculate_distance(
                self.trajectory.x[index], state)

            while True:
                next_distance = self._calculate_distance(
                    self.trajectory.x[index + 1], state
                )

                if this_distance < next_distance:
                    break

                if (index + 1) < len(self.trajectory.x):
                    index += 1

                this_distance = next_distance

            self.old_nearest_point_index = index

        v = (input[0] + input[1]) / 2

        lookahead_distance = VFHPurePursuit.lookahead_gain * v + \
            lookahead_distance

        distance = self._calculate_distance(self.trajectory.x[index], state)

        while lookahead_distance > distance:
            if index + 1 >= len(self.trajectory.x):
                break

            index += 1

            distance = self._calculate_distance(
                self.trajectory.x[index], state)

        return index, lookahead_distance
