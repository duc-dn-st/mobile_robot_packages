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

        # robot parameter
        self.max_speed = 0.27  # [m/s]

        self.min_speed = -0.27  # [m/s]

        self.max_yaw_rate = 40.0 * math.pi / 180.0  # [rad/s]

        self.max_accel = 0.27  # [m/ss]
        
        self.max_delta_yaw_rate = 40.0 * math.pi / 180.0  # [rad/ss]
        
        self.v_resolution = 0.03  # [m/s]
        
        self.yaw_rate_resolution = 1.0 * math.pi / 180.0  # [rad/s]
        
        # DWA parameter
        self.dt = 0.1  # [s] Time tick for motion prediction
        
        self.predict_time = 2.0  # [s]
        
        self.to_goal_cost_gain = 0.15
        
        self.speed_cost_gain = 1.0
        
        self.goal_index = 0

        self.goal = self.trajectory.x[self.goal_index]

        self.goal_tolerance = 0.75  # [m]

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

        goal_distance = math.sqrt((self.goal[0] - state[0])**2 + (self.goal[1] - state[1])**2)

        if goal_distance <= self.goal_tolerance:
        
            self.goal_index += 1

        if self.goal_index >= len(self.trajectory.x):
            
            u = [0.0, 0.0]
        
            return status, u                 

        self.goal = self.trajectory.x[self.goal_index]

        u, predicted_trajectory = self._dwa_control(state, self.goal, input)

        return status, u

    # ==================================================================================================
    # PRIVATE METHODS
    # ==================================================================================================
    def _dwa_control(self, state, goal, input):
        """! This function is used to calculate the control.
        @param state<list>: The state of the vehicle.
        @param goal<list>: The goal of current step.
        @param input<list>: The input of the vehicle.
        """
        dw = self._calc_dynamic_window(input)

        u, trajectory = self._calc_control_and_trajectory(state, dw, goal)

        print("u: ", u)

        return u, trajectory
    
    def _calc_dynamic_window(self, input):
        """! This function is used to calculate the dynamic window. 
        @param state<list>: The state of the vehicle.
        @param input<list>: The input of the vehicle.
        """
        Vs = [self.min_speed, self.max_speed,
              -self.max_yaw_rate, self.max_yaw_rate]
        
        Vd = [input[0] - self.max_accel * self.dt,
              input[0] + self.max_accel * self.dt,
              input[1] - self.max_delta_yaw_rate * self.dt, 
              input[1] + self.max_delta_yaw_rate * self.dt]
        
        dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
              max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
        
        return dw
    
    def _predict_trajectory(self, state, v, y):
        """! This function uses the kinematic model. 
        Predict the state of the robot in self.predict_time 
        using the control [v, y].
        @param state<list>: The state of the vehicle.
        @param v<float>: The speed of the vehicle.
        @param y<float>: The yaw rate of the vehicle.
        """
        trajectory = np.array(state)

        time = 0

        while time <= self.predict_time:

            state = self.model.function(state, [v, y], self.dt)
        
            trajectory = np.vstack((trajectory, state))
        
            time += self.dt

        return trajectory
    
    def _calculate_goal_cost(self, trajectory, goal):
        """! This function is used to calculate the goal cost.
        @param trajectory<list>: The trajectory of the vehicle.
        @param goal<list>: The goal of current step.
        """

        dx = goal[0] - trajectory[-1, 0]
        dy = goal[1] - trajectory[-1, 1]
        error_angle = math.atan2(dy, dx)
        cost_angle = error_angle - trajectory[-1, 2]
        cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

        return cost

    def _calc_control_and_trajectory(self, state, dw, goal):
        """! This function is used to calculate the control and trajectory.
        The best control of all sample is the one that minimize the cost function.
        Cost function = goal_cost + speed_cost

        @param state<list>: The state of the vehicle.
        @param dw<list>: The dynamic window.
        @param goal<list>: The goal of current step.
        """

        min_cost = float("inf")

        best_u = [0.0, 0.0]
        
        best_trajectory = np.array([state])

        for v in np.arange(dw[0], dw[1], self.v_resolution):

            for y in np.arange(dw[2], dw[3], self.yaw_rate_resolution):

                trajectory = self._predict_trajectory(state, v, y)                

                to_goal_cost = self.to_goal_cost_gain * self._calculate_goal_cost(trajectory , goal) 
        
                speed_cost = self.speed_cost_gain * (self.max_speed - v)

                final_cost = to_goal_cost + speed_cost

                if min_cost >= final_cost:

                    min_cost = final_cost
        
                    best_u = [v, y]
        
                    best_trajectory = trajectory

        return best_u, best_trajectory