#!/usr/bin/env python3
## 
# @file lmpc.py
#
# @brief Provide implementation of Linear Model Predictive Control (L-MPC) controller for
# autonomous driving.
#
# @section author_doxygen_example Author(s)
# - Created by Dinh Ngoc Duc on 29/08/2024.

# Standard library
import math
import numpy as np
from scipy.linalg import block_diag
import cvxpy as cp

class LinearModelPredictiveControl:
    """! Linear Model Predictive Control (L-MPC) controller
    The class provides implementation of Linear Model Predictive (L-MPC) 
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

        self._nx = 3

        self._nu = 2

        self._predict_step = 10

    

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

        goal_distance = math.sqrt((self._goal[0] - state[0])**2 + (self._goal[1] - state[1])**2)

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

    def _convert_degrees_to_radians(self, degrees):
        """! Convert degrees to radians
        @param degrees<float>: The angle in degrees
        """
        radians = degrees * (math.pi / 180)
        
        return radians
 
    def _execute_mpc_control(self, state, goal, input):
        """! This function is used to calculate the control.
        @param state<list>: The state of the vehicle.
        @param goal<list>: The goal of current step.
        @param input<list>: The input of the vehicle.
        """
        dw = self._calculate_dynamic_window(input)

        u, trajectory = self._calculate_control_and_trajectory(state, dw, goal)

        print("u: ", u)

        return u
    
    def _set_stage_cost(self):
        """! This function is used to calculate the cost function.
        @param state<list>: The state of the vehicle.
        @param goal<list>: The goal of current step.
        @param input<list>: The input of the vehicle.
        """ 
        Q = np.diag([200, 200, 20, 0.001])

        R = np.diag([0.001, 0.001])
        
        # Stack Q self._predict_step+1 times
        Q_stacked = block_diag(*([Q] * (self._predict_step+1)))

        R_stacked = block_diag(*([R] * self._predict_step))

        H = np.diag(Q_stacked, R_stacked)

        return H

    def _set_inquality_connstratints(self):
        """! 
        """

        G = np.matrix([[1, 0, 1, 0], 
                          [1, 0, 1, 0], 
                          [0, 1, 0, 1], 
                          [0, 1, 0, 1]])
        
        h = np.matrix([[self.model.velocity_max], 
                       [-self.model.velocity_max], 
                       [self.model.angular_velocity_max], 
                       [-self.model.angular_velocity_max]])
        
        A_in = np.zeros((0, (self._predict_step+1)*self._nx + self._predict_step*self._nu))

        B_in = np.zeros((0, 1))

        for i in range(self._predict_step-1):

            A_temp = np.hstack((np.zeros([G.shape[0] , self._nx*(self._predict_step+1)]), np.zeros([G.shape[0],i*self._nu]),  G, np.zeros([G.shape[0], self._predict_step*self._nu - G.shape[1] - i*self._nu])))
    
            A_in = np.vstack((A_in, A_temp))

            B_in = np.vstack((B_in, h))

        return A_in, B_in
        

    def _set_equality_constraints(self, state_ref, input_ref):
        
        A_eq = np.zeros((0, self._nx*(self._predict_step+1) + self._nx*self._predict_step))

        B_eq = np.zeros((self._nx*(self._predict_step), 1))

        for i in range(self._predict_step):
            
            A_d, B_d = self.model.get_state_space_matrices(state_ref[i], input_ref[i], self._dt)
        
            A_temp = np.hstack((np.zeros((self._nx, i*self._nx)), A_d, -np.identity(self._nx), np.zeros((self._nx, self._nx*(self._predict_step-i-1)))))
            
            B_temp = np.hstack((i*np.zeros((self._nx, i*self._nu)), B_d, np.zeros((self._nx, self._nu*(self._predict_step-i-1)))))
            
            A_eq_temp = np.hstack((A_temp, B_temp))
            
            A_eq = np.vstack((A_eq, A_eq_temp))
        
        return A_eq, B_eq
        

    def _set_optimization_problem(self):

        z = cp.Variable(self._nx*(self._predict_step+1) + self._nu*self._predict_step)

        H = self._set_stage_cost(self._predict_step)

        A_eq, B_eq = self._set_equality_constraints(state_ref, input_ref)

        A_in, B_in = self._set_inquality_connstratints()

        q = np.zeros((self._nx*(self._predict_step+1) + self._nu*self._predict_step, 1))

        obj = cp.Minimize((1/2)*cp.quad_form(z, H) + q.T @ z)

        constraints = [A_in @ z <= B_in, A_eq @ z == B_eq]
        
        prob = cp.Problem(obj,constraints)