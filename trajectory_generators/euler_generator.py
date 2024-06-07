# Standard library
import math
import numpy as np

# Internal library 
from trajectory_generators.a_star import AStar

ACCELERATION_LIMIT = 0.177

JERK_LIMIT = ACCELERATION_LIMIT / (40 * 0.05)


class EulerGenerator: 
    def __init__(self, environment):
        self._n = 50 

        self._generate_coefficient()

        self._a_star = AStar(environment)

        self._wheel_base = 0.255

        self._epsilon_max = 0.153281

        self._lc_scale = 0.4

        self._velocity_max = 10 # m/s

        self._sampling_time = 0.05

    def generator(self, start, end):
        initial_paths, length = self._a_star.generate(start, end)

        self.a_star_paths = initial_paths
        
        if length <= 2: 
            return initial_paths, length
        
        number_of_corner = len(initial_paths) - 2

        number_of_line = len(initial_paths) - 1

        start_corners, mid_corners, end_corners, path_velocity_corners, times, a_eulers, turn_direction = self._generate_euler_spiral(number_of_corner, initial_paths)

        start = np.append(np.array([initial_paths[0]]), end_corners, axis=0)

        end = np.append(start_corners, np.array([initial_paths[-1]]), axis=0)
        
        euler_paths = []

        for index in range(number_of_corner):
            euler_paths.append(list(initial_paths[index]))

            euler_paths.append(list(start_corners[index]))

            euler_paths.append(list(mid_corners[index]))

            euler_paths.append(list(end_corners[index]))

            euler_paths.append(list(initial_paths[index + 2]))

        position = self._interpolate(number_of_line, start, start_corners, end, path_velocity_corners, times, a_eulers, turn_direction)
        
        return position, euler_paths

    def _interpolate(self, number_of_line, start, start_corners, end, path_velocity_corners, times, a_eulers, turn_direction):
        unit_vector = np.zeros((number_of_line, 2))

        position = []

        for index in range(number_of_line - 1):
            line_length = np.linalg.norm(end[index, :] - start[index, :])

            unit_vector[index] = (end[index, :] - start[index, :]) / line_length

            # if index > number_of_line - 1: 
            #     return position

            theta_start = self._calculate_angle(unit_vector[index])

            rotation_matrix = np.array([[math.cos(theta_start), -math.sin(theta_start)], [math.sin(theta_start), math.cos(theta_start)]])

            arc_length_mid = path_velocity_corners[index] * (1.0 / 2) * times[index]

            phi_mid_local  = (math.pi / 2) * pow((arc_length_mid / a_eulers[index]), 2)

            number_of_points = int(times[index] / self._sampling_time)

            for j in range(number_of_points):
                arc_length = path_velocity_corners[index] * (1.0 / 2) * (times[index] - j * self._sampling_time)

                if j < number_of_points / 2 - 1:
                    phi = (math.pi / 2) * pow((arc_length / a_eulers[index]), 2)

                    unit = self._generate_euler_spiral_coordinate(phi)

                    local = a_eulers[index] * np.array([unit[0], unit[1] * turn_direction[index]])

                else: 
                    phi = phi_mid_local + (math.pi / pow(a_eulers[index], 2)) * (-(1.0 / 2) * pow(arc_length, 2) + 2 * arc_length_mid * arc_length - (3.0 / 2) * pow(arc_length_mid, 2))

                    unit = self._generate_euler_spiral_coordinate(abs(phi))
                    
                    local = a_eulers[index] * np.array([unit[0], unit[1] * turn_direction[index]])

                position.append(np.matmul(rotation_matrix, local.transpose()).transpose() + start_corners[index]) 

        return position
        
    def _generate_euler_spiral(self, number_of_corner, initial_paths):
        waypoints = self._convert_to_numpy(initial_paths)

        epsilon = np.zeros(number_of_corner)

        start_corners = np.zeros((number_of_corner, 2))

        mid_corners = np.zeros((number_of_corner, 2))

        end_corners = np.zeros((number_of_corner, 2))

        path_velocity_corners = np.zeros(number_of_corner)

        times = np.zeros(number_of_corner)

        a_euler = np.zeros(number_of_corner)

        lc = np.zeros(number_of_corner)

        turn_direction = np.zeros(number_of_corner)

        for index in range(number_of_corner):
            start = waypoints[index]

            corner = waypoints[index + 1]

            end = waypoints[index + 2]

            delta_start = corner - start

            unit_start = delta_start / np.linalg.norm(delta_start)

            theta_start = self._calculate_angle(unit_start)

            delta_end = end - corner

            unit_end = delta_end / np.linalg.norm(delta_end)

            theta_end = self._calculate_angle(unit_end)

            theta_end_local = theta_end - theta_start

            if theta_end_local > math.pi: 
                theta_end_local = self._wrap_to_pi(theta_end_local)

            if theta_end_local > 0:
                turn_direction[index] = 1
            
            else:
                turn_direction[index] = -1

            delta_phi = abs(theta_end_local)

            beta = math.pi - delta_phi

            theta_epsilon_local = (delta_phi + 0.5 * beta) * turn_direction[index]

            phi_mid = 0.5 * delta_phi

            unit_mid = self._generate_euler_spiral_coordinate(phi_mid)

            a_euler_min = self._wheel_base * math.sqrt(2 * math.pi * phi_mid)

            # epsilon_min = a_euler_min * unit_mid[1] / math.sin(beta / 2)

            epsilon[index] = self._epsilon_max

            lc[index] = epsilon[index] * ((math.sin(beta / 2) / unit_mid[1]) * unit_mid[0] + math.cos(beta / 2))

            a_euler[index] = (epsilon[index] * math.sin(beta / 2)) / unit_mid[1]

            lc_check = [np.linalg.norm(delta_end), np.linalg.norm(delta_start)]

            if lc[index] > self._lc_scale * np.min(lc_check):
                lc[index] = self._lc_scale * np.min(lc_check)

                epsilon[index] = lc[index] / ((math.sin(beta / 2) / unit_mid[1] * unit_mid[0] + math.cos(beta / 2)))

                a_euler[index] = (epsilon[index] * math.sin(beta / 2)) / unit_mid[1]

            if a_euler_min > a_euler[index]:
                a_euler[index] = a_euler_min

                epsilon[index] = a_euler_min * unit_mid[1] / (math.sin(beta / 2))

                lc[index] = epsilon[index] * ((math.sin(beta / 2) / unit_mid[1]) * unit_mid[0] + math.cos(beta / 2))

            rotation_matrix = np.array([[math.cos(theta_start), -math.sin(theta_start)], [math.sin(theta_start), math.cos(theta_start)]])

            corner_local = np.matmul(rotation_matrix.transpose(), delta_start.transpose()).transpose()
            
            start_corner_local = corner_local + lc[index] * np.array([-1, 0])

            epsilon_corner_local = corner_local + epsilon[index] *  np.array([math.cos(theta_epsilon_local), math.sin(theta_epsilon_local)])
            
            end_corner_local = corner_local + lc[index] * np.array([math.cos(theta_end_local), math.sin(theta_end_local)])

            start_corners[index] = np.matmul(rotation_matrix, start_corner_local.transpose()).transpose() + start

            mid_corners[index] = np.matmul(rotation_matrix, epsilon_corner_local.transpose()).transpose() + start

            end_corners[index] = np.matmul(rotation_matrix, end_corner_local.transpose()).transpose() + start

            path_velocity_corners[index] = 0.2 # m/s

            s_mid = a_euler[index] * math.sqrt(2 * phi_mid / math.pi)

            times[index] = 2 * s_mid / path_velocity_corners[index]

        return start_corners, mid_corners, end_corners, path_velocity_corners, times, a_euler, turn_direction

    def _generate_euler_spiral_coordinate(self, angle): 
        unit = np.array([0.0, 0.0])

        for index in range(self._n):
            first_exp = 0.5 * (4 * index + 1)

            second_exp = 0.5 * (4 * index + 3)

            unit[0] = unit[0] + self._euler_table[index, 0] * math.pow(angle, first_exp)

            unit[1] = unit[1] + self._euler_table[index, 1] * math.pow(angle, second_exp)

        unit = unit / math.sqrt(2 * math.pi)

        return unit

    @staticmethod
    def _wrap_to_pi(angle):
        return angle - 2 * math.pi * math.floor((angle + math.pi) / (2 * math.pi))
        
    def _calculate_angle(self, unit_array):
        angle = 0

        unit_array_x = abs(unit_array[0])

        if unit_array[1] >= 0:
            if self._is_same(unit_array[0], 0):
                angle = 0.5 * math.pi

            else: 
                angle = math.acos(unit_array[0])

        else: 
            if self._is_same(unit_array[0], 0):
                angle = -0.5 * math.pi
            
            elif unit_array[0] < 0:
                angle = -math.pi + math.acos(unit_array_x)

            else: 
                angle = -math.acos(unit_array_x)
        
        return angle
    
    @staticmethod
    def _is_same(lhs, rhs):
        return abs(lhs - rhs) < 1e-9

    def _convert_to_numpy(self, list_of_tuple):
        output = []

        for tuple in list_of_tuple:
            output.append(np.array(tuple))

        return output

    def _generate_coefficient(self):
        self._euler_table = np.zeros((self._n, 2))

        for i in range(self._n):
            factorial_x = 1 

            if i > 0: 
                for j in range(1, 2 * i + 1):
                    factorial_x = factorial_x * j

            factorial_y = factorial_x * (2 * i + 1)

            self._euler_table[i, 0] = pow(-1, i) * 2 / (factorial_x * ((4 * i) + 1))

            self._euler_table[i, 1] = pow(-1, i) * 2 / (factorial_y * ((4 * i) + 3))