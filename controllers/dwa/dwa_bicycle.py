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

    # =================================================================================================
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

        if self._is_goal(state, self.trajectory):
            
            return False, [0, 0]

        goal = self._search_target_index(state)

        u = self._execute_dwa_control(state, goal, input)

        return status, u

    # ==================================================================================================
    # PRIVATE METHODS
    # ==================================================================================================

    def _register_model_specification(self):
        """! Register the model specification
        @note The method is used to register the model specification.
        """
        self._min_speed = 0.0

        self._max_speed = self.model.velocity_max

        self._delta_min = math.radians(-40.0)

        self._delta_max = math.radians(40.0)

        self._max_acceleration = 2.0

        self._max_delta_dot = math.radians(80)

        self._v_resolution = 0.1

        self._delta_resolution = math.radians(1.0)

    def _register_DWA_specification(self):
        """! Register the DWA specification
        @note The method is used to register the DWA specification.
        @ self._lookahead_time : minimum look ahead time :
            1->3 : well for precise tracking ,
            5->10 : better anticipation of obstacles and smoother motion
        """
        self._dt = self.trajectory.sampling_time

        self._lookahead_steps = 2

        self._lookahead_time = self._lookahead_steps * self._dt

        self._to_goal_cost_gain = 1.0

        self._speed_cost_gain = 1.0

        self._goal_tolerance = 0.3

    def _find_nearest_waypoint(self, state):
        """! Get the nearest waypoint
        @param state<list>: The state of the vehicle
        @param index<int>: The previous index
        """
        dx = [state[0] - ref_x for ref_x in self.trajectory.x[:, 0]]

        dy = [state[1] - ref_y for ref_y in self.trajectory.x[:, 1]]

        d = [math.sqrt(idx ** 2 + idy ** 2) for (idx, idy) in zip(dx, dy)]

        min_d = min(d)

        nearest_index = d.index(min_d)

        return nearest_index

    def _search_target_index(self, state):
        """! Get the tracking goal
        @param state<list>: The state of the vehicle
        """
        self.idx = self._find_nearest_waypoint(state)

        goal = self.trajectory.x[self.idx]

        goal_distance = self._calculate_distance(state, goal)

        while goal_distance <= self._goal_tolerance:

            if self.idx + 1 < len(self.trajectory.x):

                self.idx += 1

            else:
                break

            goal = self.trajectory.x[self.idx]

            goal_distance = self._calculate_distance(state, goal)

        if self.idx + self._lookahead_steps < len(self.trajectory.x):

            goal = self.trajectory.x[self.idx + self._lookahead_steps]

        else:

            goal = self.trajectory.x[-1]

        return goal

    def _execute_dwa_control(self, state, goal, input):
        """! This function is used to calculate the control.
        @param state<list>: The state of the vehicle.
        @param goal<list>: The goal of current step.
        @param input<list>: The input of the vehicle.
        """
        dw = self._calculate_dynamic_window(input)

        [v, delta] = self._calculate_control(state, input, dw, goal)

        return [v, delta]

    def _calculate_dynamic_window(self, input):
        """! This function is used to calculate the dynamic window.
        @param state<list>: The state of the vehicle.
        @param input<list>: The input of the vehicle.
        """
        Vs = [self._min_speed, self._max_speed,
              self._delta_min, self._delta_max]

        Vd = [input[0] - self._max_acceleration * self._dt,
              input[0] + self._max_acceleration * self._dt,
              input[1] - self._max_delta_dot * self._dt,
              input[1] + self._max_delta_dot * self._dt]

        dw = [max(Vs[0], Vd[0]),
              min(Vs[1], Vd[1]),
              max(Vs[2], Vd[2]),
              min(Vs[3], Vd[3])]

        return dw

    def _calculate_control(self, state, input, dw, goal):
        """! This function is used to calculate the control input
        The best control of all sample is the one that minimize the cost
        function.
        @param state<list>: The state of the vehicle.
        @param input<list>: The input of the vehicle.
        @param dw<list>: The dynamic window.
        @param goal<list>: The goal of current step.
        """
        min_cost = float("inf")

        for v in np.arange(dw[0], dw[1], self._v_resolution):

            for delta in np.arange(dw[2], dw[3], self._delta_resolution):

                lookahead_trajectory = self._predict_trajectory(
                    state, [v, delta])

                to_goal_cost = self._to_goal_cost_gain * \
                    self._calculate_to_goal_cost(lookahead_trajectory, goal)

                speed_cost = self._speed_cost_gain * \
                    self._calculate_speed_cost([v, delta])

                final_cost = speed_cost + to_goal_cost

                if min_cost >= final_cost:

                    min_cost = final_cost

                    best_v = v
                    
                    best_delta = delta

        return [best_v, best_delta]

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

    def _calculate_to_goal_cost(self, predict_trajectory, goal):
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

    def _calculate_speed_cost(self, input):
        """! This function is used to calculate the goal cost.
        """
        speed_cost = (self._max_speed - input[0])

        return speed_cost

    # ==================================================================================================
    # STATIC METHODS
    # ==================================================================================================
    @staticmethod
    def _is_goal(state, trajectory):
        """! Check if the vehicle has reached the goal
        @param state<list>: The state of the vehicle
        @param trajectory<instance>: The trajectory
        @return<bool>: The flag to indicate if the vehicle has reached the goal
        """
        delta_x = trajectory.x[-1, 0] - state[0]

        delta_y = trajectory.x[-1, 1] - state[1]

        distance = np.hypot(delta_x, delta_y)

        return distance < 0.1

    @staticmethod
    def _calculate_distance(state, reference):
        """! Calculate the distance to the goal
        @param state<list>: The state of the vehicle
        @param goal<list>: The goal of current step
        """
        dx = reference[0] - state[0]

        dy = reference[1] - state[1]

        return math.hypot(dx, dy)
