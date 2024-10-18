#!/usr/bin/env python3
##
# @file dwa.py
#
# @brief Provide implementation of Dynamic Window Approach (DWA) controller for
# autonomous driving.
#
# @section author_doxygen_example Author(s)
# - Created by Dinh Ngoc Duc on 24/08/2024.

# Standard library
import math
import numpy as np
import matplotlib.pyplot as plt


class DynamicWindowApproach:
    """! Dynamic Window Approach (DWA) controller
    The class provides implementation of Dynamic Window Approach (DWA)
    controller for autonomous driving.
    @note The controller is used to follow a trajectory.
    """

    def __init__(self, model, trajectory):
        """! Constructor
        @param model<instance>: The vehicle model
        @param trajectory<instance>: The trajectory
        @note The trajectory is a set of point to point.
        """
        self.model = model

        self.trajectory = trajectory

        self._register_model_specification()

        self._register_DWA_specification()

    # ==================================================================================================
    # PUBLIC METHODS
    # =================================================================================================

    def initialize(self):
        """! Initialize the controller
        @note The method is used to initialize the controller.
        """
        pass

    def execute(self, state, input, index):
        """! Execute the controller
        @param state<list>: The state of the vehicle
        @param input<list>: The input of the vehicle
        @param index<int>: The previous index
        @return<tuple>: The status and control
        """
        status = True

        goal_distance = math.sqrt(
            (self._goal[0] - state[0])**2 + (self._goal[1] - state[1])**2)

        if goal_distance <= self._goal_tolerance:

            self._goal_index += 1

        if self._goal_index >= len(self.trajectory.x):

            u = [0.0, 0.0]

            return status, u

        self._goal = self.trajectory.x[self._goal_index]

        u = self._execute_dwa_control(state, self._goal, input)

        return status, u

    # ==================================================================================================
    # PRIVATE METHODS
    # ==================================================================================================

    def _register_model_specification(self):
        """! Register the model specification
        @note The method is used to register the model specification.
        """
        # Max and min velocity
        self._max_speed = self.model.velocity_max # [m/s]

        self._delta_max = self._convert_degrees_to_radians(40.0) # [rad]

        self._delta_min = self._convert_degrees_to_radians(-40.0) # [rad]

        # Max and min acceleration
        self._max_acceleration = self.model.velocity_max # [m/s^2]

        self._max_delta_dot = self._convert_degrees_to_radians(20) # [rad/s]

        # Resolution

        self._v_resolution = 0.1 # [m/s]

        self._delta_resolution = self._convert_degrees_to_radians(1.0) # [rad]

    def _register_DWA_specification(self):
        """! Register the DWA specification
        @note The method is used to register the DWA specification.
        @ self._lookahead_time : minimum look ahead time : 
            1->3 : well for precise tracking , 
            5->10 : better anticipation of obstacles and smoother motion in faster
        """
        self._dt = self.trajectory.sampling_time

        self._lookahead_time = 3 * self._dt 

        # NOTE : TUNING LATER 
        self._orientation_to_goal_cost_gain = 0.5
 
        self._speed_cost_gain = 0.5

        self._goal_index = 1

        self._goal = self.trajectory.x[self._goal_index]

        # NOTE : NOT SURE
        # self._goal_tolerance = (self._max_speed * self._dt) 

        self._goal_tolerance = 0.5 


    def _execute_dwa_control(self, state, goal, input):
        """! This function is used to calculate the control.
        @param state<list>: The state of the vehicle.
        @param goal<list>: The goal of current step.
        @param input<list>: The input of the vehicle.
        """
        dw = self._calculate_dynamic_window(input)

        u = self._calculate_control_and_trajectory(state, dw, goal)

        return u

    def _calculate_dynamic_window(self, input):
        """! This function is used to calculate the dynamic window.
        @param state<list>: The state of the vehicle.
        @param input<list>: The input of the vehicle.
        """
        Vs = [0, self._max_speed,
              self._delta_min, self._delta_max]

        Vd = [input[0] - self._max_acceleration * self._dt,
              input[0] + self._max_acceleration * self._dt,
              input[1] - self._max_delta_dot * self._dt,
              input[1] + self._max_delta_dot * self._dt]

        dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
              max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

        return dw

    def _calculate_control_and_trajectory(self, state, dw, goal):
        """! This function is used to calculate the control and trajectory.
        The best control of all sample is the one that minimize the cost
        function.
        Cost function = goal_cost + speed_cost

        @param state<list>: The state of the vehicle.
        @param dw<list>: The dynamic window.
        @param goal<list>: The goal of current step.
        """
        min_cost = float("inf")

        best_u = [0.0, 0.0]

        best_trajectory = None

        for v in np.arange(dw[0], dw[1], self._v_resolution):

            for delta in np.arange(dw[2], dw[3], self._delta_resolution):

                input = [v, delta]

                trajectory = self._predict_trajectory(state, input)

                orientation_to_goal_cost = self._orientation_to_goal_cost_gain * \
                    self._calculate_orientation_to_goal_cost(trajectory, goal)

                speed_cost = self._speed_cost_gain * (self._max_speed - v)

                final_cost = speed_cost + orientation_to_goal_cost

                if min_cost >= final_cost:

                    min_cost = final_cost

                    best_u = input

                    best_trajectory = trajectory

        return best_u

    def _predict_trajectory(self, state, input):
        """! This function uses the kinematic model.
        Predict the state of the robot in self._lookahead_time
        using the control input.
        @param state<list>: The state of the vehicle.
        @param v<float>: The speed of the vehicle.
        @param y<float>: The yaw rate of the vehicle.
        """
        trajectory = np.array(state)

        time = 0

        while time <= self._lookahead_time:

            state = self.model.function(state, input, self._dt)

            trajectory = np.vstack((trajectory, state))

            time += self._dt

        return trajectory
    
    def _calculate_orientation_to_goal_cost(self, predict_trajectory, goal):
        """! This function is used to calculate the goal cost.
        @param trajectory<list>: The trajectory of the vehicle.
        @param goal<list>: The goal of current step.
        """
        dx = goal[0] - predict_trajectory[-1, 0]
        dy = goal[1] - predict_trajectory[-1, 1]
        error_angle = math.atan2(dy, dx)
        cost_angle = error_angle - predict_trajectory[-1, 2]
        cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

        return cost

    def _calculate_tracking_error_cost(self, state, goal):
        """! This function is used to calculate the goal cost.
        @param trajectory<list>: The trajectory of the vehicle.
        @param goal<list>: The goal of current step.
        """
        dx = goal[0] - state[0]
        dy = goal[1] - state[1]
        error_distance = math.sqrt(dx**2 + dy**2)
        cost = error_distance

        return cost

    # ==================================================================================================
    # OTHER METHODS
    # ==================================================================================================
    def find_lookahead_index(robot_pos, trajectory, lookahead_distance):
        # Calculate the distance from the robot to each waypoint in the trajectory
        lookahead_distance = 0.5
        for i in range(len(trajectory)):
            dx = trajectory[i, 0] - robot_pos[0]
            dy = trajectory[i, 1] - robot_pos[1]
            distance = math.sqrt(dx**2 + dy**2)
            
            # Find the first waypoint that is at least lookahead_distance away
            if distance >= lookahead_distance:
                return i
        
        # If no such waypoint is found, return the last index
        return len(trajectory) - 1

    def _find_nearest_waypoint(self, state, index, update_prev_idx=False):
        """! Get the nearest waypoint
        @param state<list>: The state of the vehicle
        @param index<int>: The previous index
        @param update_prev_idx<bool>: The flag to update the previous index
        """
        SEARCH_IDX_LEN = 200

        dx = [state[0] - ref_x for ref_x in self.trajectory.x[index:(index + SEARCH_IDX_LEN), 0]]

        dy = [state[1] - ref_y for ref_y in self.trajectory.x[index:(index + SEARCH_IDX_LEN), 1]]

        d = [math.sqrt(idx ** 2 + idy ** 2) for (idx, idy) in zip(dx, dy)]

        min_d = min(d)

        nearest_idx = d.index(min_d) + index

        return nearest_idx

    def _convert_degrees_to_radians(self, degrees):
        """! Convert degrees to radians
        @param degrees<float>: The angle in degrees
        """
        radians = degrees * (math.pi / 180)

        return radians
