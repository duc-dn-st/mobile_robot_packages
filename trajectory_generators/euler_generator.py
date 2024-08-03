#!/usr/bin/env python3
##
# @file euler_generator.py
#
# @brief Provide euler generator for the path planning
#
# @section author_doxygen_example Author(s)
# - Created by Tran Viet Thanh on 18/06/2024.
#
# Copyright (c) 2024 System Engineering Laboratory.  All rights reserved.

# Standard library
import math
import numpy as np

ACCELERATION_LIMIT = 0.177

JERK_LIMIT = ACCELERATION_LIMIT / (40 * 0.05)


class EulerGenerator:
    """! Provides a generator for Euler spiral paths"""

    # ==================================================================================================
    # PUBLIC METHODS
    # ==================================================================================================
    def __init__(self, model, sampling_time=0.05):
        """! The constructor for the EulerGenerator class
        @param model<Model>: The model of the vehicle
        @param sampling_time<float>: The sampling time
        @return The instance of the EulerGenerator class
        """
        self._n = 50

        self._euler_coefficient = self._calculate_coeffient()

        self._model = model

        self._epsilon_max = 0.21425

        self._lc_scale = 0.4

        self._sampling_time = sampling_time

    def generate(self, initial_paths):
        """! Generate the Euler spiral path
        @param initial_paths<list>: The initial paths
        @return The Euler spiral path
        """
        if len(initial_paths) <= 2:
            raise ValueError("The number of paths must be greater than 2")

        trajectory = []

        number_of_corner = len(initial_paths) - 2

        waypoints = self._convert_to_numpy(initial_paths)

        previous_end_corner = initial_paths[0]

        for index in range(number_of_corner):
            start = waypoints[index]

            corner = waypoints[index + 1]

            end = waypoints[index + 2]

            theta_start = self._calculate_angle_from_vectors(start, corner)

            theta_end = self._calculate_angle_from_vectors(corner, end)

            theta_end_local = self._wrap_to_pi(theta_end - theta_start)

            turn_direction = 1 if theta_end_local > 0 else -1

            lc, a_euler, s_mid = self._calculate_curve_parameters(
                start, corner, end, theta_end_local
            )

            start_corner, end_corner = self._calculate_corner_points(
                start, corner, theta_end_local, lc
            )

            path_velocity_corner = self._model.velocity_max  # m/s

            time = 2 * s_mid / path_velocity_corner

            start = initial_paths[index] if index == 0 else previous_end_corner

            end = (
                initial_paths[index]
                if index == len(initial_paths) - 1
                else start_corner
            )

            previous_end_corner = end_corner

            position = self._interpolate(
                start_corner,
                path_velocity_corner,
                time,
                a_euler,
                turn_direction,
                start,
                end,
            )

            trajectory.extend(position)

        return trajectory

    # ==================================================================================================
    # PRIVATE METHODS
    # ==================================================================================================
    def _interpolate(
        self,
        start_corner,
        path_velocity_corner,
        time,
        a_euler,
        turn_direction,
        start,
        end,
    ):
        """! Interpolate the Euler spiral path
        @param start_corner<np.array>: The start corner
        @param path_velocity_corner<float>: The path velocity corner
        @param time<float>: The time
        @param a_euler<float>: The a euler
        @param turn_direction<int>: The turn direction
        @param start<np.array>: The start position
        @param end<np.array>: The end position
        @return The interpolated Euler spiral path
        """
        # if index > number_of_line - 1:
        #     return position
        theta_start = self._calculate_angle_from_vectors(start, end)

        rotation_matrix = np.array(
            [
                [math.cos(theta_start), -math.sin(theta_start)],
                [math.sin(theta_start), math.cos(theta_start)],
            ]
        )

        arc_length_mid = path_velocity_corner * (1.0 / 2) * time

        phi_mid_local = (math.pi / 2) * pow((arc_length_mid / a_euler), 2)

        number_of_points = math.ceil(time / self._sampling_time)

        position = []

        for j in range(1, number_of_points + 1):
            arc_length = path_velocity_corner * j * self._sampling_time

            if j * self._sampling_time <= time / 2:
                phi = (math.pi / 2) * pow((arc_length / a_euler), 2)

                unit = self._generate_euler_spiral_coordinate(phi)

                dphi_local = (
                    turn_direction
                    * math.pi
                    * path_velocity_corner
                    * arc_length
                    / math.pow(a_euler, 2)
                )

                local = a_euler * np.array([unit[0], unit[1] * turn_direction])

            else:
                phi = phi_mid_local + (math.pi / pow(a_euler, 2)) * (
                    -(1.0 / 2) * pow(arc_length, 2)
                    + 2 * arc_length_mid * arc_length
                    - (3.0 / 2) * pow(arc_length_mid, 2)
                )

                dphi_local = (
                    turn_direction
                    * math.pi
                    * path_velocity_corner
                    * (2 * arc_length_mid - arc_length)
                    / math.pow(a_euler, 2)
                )

                phi_local = turn_direction * phi

                velocity_local = path_velocity_corner * np.array(
                    [math.cos(phi_local), math.sin(phi_local)]
                )

                acceleration_local = (
                    path_velocity_corner
                    * dphi_local
                    * np.array([-math.sin(phi_local), math.cos(phi_local)])
                )

                local = (
                    local
                    + velocity_local * self._sampling_time
                    + (1.0 / 2) * acceleration_local * math.pow(
                        self._sampling_time, 2)
                )

            position.append(
                np.matmul(rotation_matrix, local.transpose()
                          ).transpose() + start_corner
            )

        return position

    def _calculate_corner_points(self, start, corner, theta_end_local, lc):
        """! Calculate the corner points
        @param start<np.array>: The start point
        @param corner<np.array>: The corner point
        @param theta_end_local<float>: The theta end local
        @param lc<float>: The lc
        @return The corner points. Including start_corner and end_corner
        """
        theta_start = self._calculate_angle_from_vectors(start, corner)

        rotation_matrix = np.array(
            [
                [math.cos(theta_start), -math.sin(theta_start)],
                [math.sin(theta_start), math.cos(theta_start)],
            ]
        )

        corner_local = np.matmul(
            rotation_matrix.transpose(), (corner - start).transpose()
        ).transpose()

        start_corner_local = corner_local + lc * np.array([-1, 0])

        end_corner_local = corner_local + lc * np.array(
            [math.cos(theta_end_local), math.sin(theta_end_local)]
        )

        start_corner = (
            np.matmul(rotation_matrix, start_corner_local.transpose()
                      ).transpose()
            + start
        )

        end_corner = (
            np.matmul(rotation_matrix, end_corner_local.transpose()
                      ).transpose() + start
        )

        return start_corner, end_corner

    def _calculate_curve_parameters(self, start, corner, end, theta_end_local):
        """! Calculate the curve parameters
        @param start<np.array>: The start point
        @param corner<np.array>: The corner point
        @param end<np.array>: The end point
        @param theta_end_local<float>: The theta end local
        @return The curve parameters
        """
        beta = math.pi - abs(theta_end_local)

        phi_mid = 0.5 * abs(theta_end_local)

        unit_mid = self._generate_euler_spiral_coordinate(phi_mid)

        # epsilon_min = a_euler_min * unit_mid[1] / math.sin(beta / 2)

        epsilon = self._epsilon_max

        lc = epsilon * (
            (math.sin(beta / 2) / unit_mid[1]) * unit_mid[0] + math.cos(
                beta / 2)
        )

        lc_check = [np.linalg.norm(end - corner), 
                    np.linalg.norm(corner - start)]

        if lc > self._lc_scale * np.min(lc_check):
            lc = self._lc_scale * np.min(lc_check)

            epsilon = lc / (
                (math.sin(beta / 2) / unit_mid[1] * unit_mid[0] + math.cos(
                    beta / 2))
            )

        a_euler_min = (self._model.wheel_base / 2) * math.sqrt(
            2 * math.pi * phi_mid)

        a_euler = (epsilon * math.sin(beta / 2)) / unit_mid[1]

        if a_euler_min > a_euler:
            a_euler = a_euler_min

            epsilon = a_euler_min * unit_mid[1] / (math.sin(beta / 2))

            lc = epsilon * (
                (math.sin(beta / 2) / unit_mid[1]) * unit_mid[0] + math.cos(
                    beta / 2)
            )

        s_mid = a_euler * math.sqrt(2 * phi_mid / math.pi)

        return lc, a_euler, s_mid

    def _generate_euler_spiral_coordinate(self, angle):
        """! Generate the Euler spiral coordinate
        @param angle<float>: The angle
        @return The Euler spiral coordinate
        """
        unit = np.array([0.0, 0.0])

        for index in range(self._n):
            first_exp = 0.5 * (4 * index + 1)

            second_exp = 0.5 * (4 * index + 3)

            unit[0] = unit[0] + self._euler_coefficient[index, 0] * math.pow(
                angle, first_exp
            )

            unit[1] = unit[1] + self._euler_coefficient[index, 1] * math.pow(
                angle, second_exp
            )

        unit = unit / math.sqrt(2 * math.pi)

        return unit

    # ==================================================================================================
    # STATIC METHODS
    # ==================================================================================================
    @staticmethod
    def _convert_to_numpy(list_of_tuple):
        """! Convert the list of tuple to numpy array
        @param list_of_tuple<list>: The list of tuple
        @return The numpy array
        """
        output = []

        for tuple in list_of_tuple:
            output.append(np.array(tuple))

        return output

    @staticmethod
    def _calculate_angle(unit_array):
        """! Calculate the angle
        @param unit_array<np.array>: The unit array
        @return The angle
        """
        angle = 0

        unit_array_x = abs(unit_array[0])

        if unit_array[1] >= 0:
            if EulerGenerator._is_same(unit_array[0], 0):
                angle = 0.5 * math.pi

            else:
                angle = math.acos(unit_array[0])

        else:
            if EulerGenerator._is_same(unit_array[0], 0):
                angle = -0.5 * math.pi

            elif unit_array[0] < 0:
                angle = -math.pi + math.acos(unit_array_x)

            else:
                angle = -math.acos(unit_array_x)

        return angle

    @staticmethod
    def _calculate_coeffient(n=50):
        """! Generate the coefficient
        @param n<int>: The number of coefficient, default is 50.
        @return The coefficient for the Euler spiral using 
        Taylor series expansion.
        """
        euler_coefficient = np.zeros((n, 2))

        for i in range(n):
            factorial_x = 1

            if i > 0:
                for j in range(1, 2 * i + 1):
                    factorial_x = factorial_x * j

            factorial_y = factorial_x * (2 * i + 1)

            euler_coefficient[i, 0] = pow(-1, i) * 2 / (
                factorial_x * ((4 * i) + 1))

            euler_coefficient[i, 1] = pow(-1, i) * 2 / (
                factorial_y * ((4 * i) + 3))

        return euler_coefficient

    @staticmethod
    def _calculate_unit_vector(start, end):
        """! Calculate the unit vector
        @param start<np.array>: The start point
        @param end<np.array>: The end point
        @return The unit vector
        """
        delta = end - start

        return delta / np.linalg.norm(delta)

    @staticmethod
    def _calculate_angle_from_vectors(start, end):
        """! Calculate the angle
        @param start<np.array>: The start point
        @param end<np.array>: The end point
        @return The angle
        """
        unit_vector = EulerGenerator._calculate_unit_vector(start, end)

        return EulerGenerator._calculate_angle(unit_vector)

    @staticmethod
    def _is_same(lhs, rhs):
        """! Check if two values are the same
        @param lhs<float>: The left hand side value
        @param rhs<float>: The right hand side value
        @return True if two values are the same, False otherwise
        """
        return abs(lhs - rhs) < 1e-9

    @staticmethod
    def _wrap_to_pi(angle):
        """! Wrap the angle to pi
        @param angle<float>: The angle
        @return The angle wrapped to pi
        """
        if angle > math.pi:
            angle = angle - 2 * math.pi * math.floor((
                angle + math.pi) / (2 * math.pi))

        return angle
