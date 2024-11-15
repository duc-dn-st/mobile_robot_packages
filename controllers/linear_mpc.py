#!/usr/bin/env python3
#
# @file linear_mpc.py
#
# @brief Provide implementation of Linear Model Predictive Control (L-MPC) controller for
# autonomous driving.
#
# @section author_doxygen_example Author(s)
# - Created by Dinh Ngoc Duc on 29/08/2024.

# Standard library
import math

# External library
import numpy as np
import cvxpy as cp
from scipy.linalg import block_diag
import matplotlib.pyplot as plt

# Internal library


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

        self._debug = False

    # ==================================================================================================
    # PUBLIC METHODS
    # =================================================================================================

    def initialize(self):
        """! Initialize the controller
        @note The method is used to initialize the controller.
        """
        self.H = self._set_stage_cost()

        pass

    def execute(self, state, input, index):
        """! Execute the controller
        @param state<list>: The state of the vehicle
        @param input<list>: The input of the vehicle
        @param index<int>: The previous index
        @return<tuple>: The status and control
        """
        status = True

        u_optimal = self._execute_mpc_control(state, input, index)

        if self._debug:
            self._plot_solution(state, u_optimal, index)

        return status, u_optimal

    # ==================================================================================================
    # PRIVATE METHODS
    # ==================================================================================================

    def _execute_mpc_control(self, state, input, index):
        """! This function is used to calculate the control.
        @param state<list>: The state of the vehicle.
        @param goal<list>: The goal of current step.
        @param input<list>: The input of the vehicle.
        """
        if index > len(self.trajectory.x) - self._predict_step:
            u = [0, 0]
        else:
            problem, solution = self._set_optimization_problem(
                state, input, index)

            u = self.trajectory.u[index] + solution[0, :]

        return u

    def _set_optimization_problem(self, state, input, index):
        """! This function is used to set the optimization problem.
        @param state<list>: The state of the vehicle.
        @param index
        """
        z = cp.Variable(self._nx * (self._predict_step + 1) +
                        self._nu * self._predict_step)

        current_idx = self._get_nearest_waypoint(state[0], state[1], index)

        # current_idx = index

        if current_idx > len(self.trajectory.x) - self._predict_step:
            current_idx = len(self.trajectory.x) - self._predict_step

        state_ref = self.trajectory.x[current_idx:current_idx +
                                      self._predict_step]

        input_ref = self.trajectory.u[current_idx:current_idx +
                                      self._predict_step]

        A_eq, B_eq = self._set_equality_constraints(state_ref, input_ref)

        A_in, B_in = self._set_inquality_constraints()

        object = cp.Minimize((1/2)*cp.quad_form(z, self.H))

        constraints = [(A_in @ z)[:, np.newaxis] <= B_in,
                       (A_eq @ z)[:, np.newaxis] == B_eq]

        constraints += [z[0:self._nx] ==
                        (state - self.trajectory.x[current_idx])]

        constraints += [z[self._nx*(self._predict_step+1):self._nx*(self._predict_step+1)+self._nu] == 0]

        optimization_problem = cp.Problem(object, constraints)

        optimization_problem.solve()

        state_sol = z.value[0:self._nx*(self._predict_step+1)
                            ].reshape((self._predict_step + 1, self._nx))

        input_sol = z.value[self._nx*(self._predict_step+1):].reshape((self._predict_step, self._nu))

        return optimization_problem, input_sol

    def _set_stage_cost(self):
        """! This function is used to calculate the cost function.
        @param state<list>: The state of the vehicle.
        @param goal<list>: The goal of current step.
        @param input<list>: The input of the vehicle.
        """
        Q = np.diag([100.0, 100.0, 0.01])

        R = np.diag([0.001, 0.001])

        Q_stacked = block_diag(*([Q] * (self._predict_step+1)))

        R_stacked = block_diag(*([R] * self._predict_step))

        H = self._diag_mat(Q_stacked, R_stacked)

        return H

    def _set_inquality_constraints(self):
        """! This function is used to set the inquality constraints.
        @return A_in, B_in
        @NOTE: G * z <= h
        G is the matrix of the inequality constraints
        G covers the constraints on the input of 2 control inputs
        h is the vector of the inequality constraints
        """
        G = np.matrix([[1, 0, -1, 0],
                       [-1, 0, 1, 0],
                       [0, 1, 0, -1],
                       [0, -1, 0, 1]])

        h = np.matrix([[self.model.velocity_max],
                    [self.model.velocity_max],
                    [self.model.angular_velocity_max],
                    [self.model.angular_velocity_max]])

        A_in = np.zeros((0, (self._predict_step+1)*self._nx +
                        self._predict_step*self._nu))

        B_in = np.zeros((0, 1))

        for i in range(self._predict_step-1):

            A_temp = np.hstack((np.zeros([G.shape[0], self._nx*(self._predict_step+1)]), np.zeros(
                [G.shape[0], i*self._nu]),  G, np.zeros([G.shape[0], self._predict_step*self._nu - G.shape[1] - i*self._nu])))

            A_in = np.vstack((A_in, A_temp))

            B_in = np.vstack((B_in, h))

        return A_in, B_in

    def _set_equality_constraints(self, state_ref, input_ref):
        """! This function is used to set the equality constraints.
        @param state_ref<list>: The reference state of the vehicle.
        @param input_ref<list>: The reference input of the vehicle.
        @return A_eq, B_eq
        """
        A_eq = np.zeros(
            (0, self._nx * (self._predict_step+1) + self._nu * self._predict_step))

        B_eq = np.zeros((self._nx * (self._predict_step), 1))

        for i in range(self._predict_step):

            A_d, B_d = self.model.get_state_space_matrices(
                state_ref[i], input_ref[i], self.trajectory.sampling_time)

            A_temp = np.hstack((np.zeros((self._nx, i*self._nx)), A_d, -np.identity(
                self._nx), np.zeros((self._nx, self._nx*(self._predict_step-i-1)))))

            B_temp = np.hstack((i*np.zeros((self._nx, i*self._nu)), B_d,
                               np.zeros((self._nx, self._nu*(self._predict_step-i-1)))))

            A_eq_temp = np.hstack((A_temp, B_temp))

            A_eq = np.vstack((A_eq, A_eq_temp))

        return A_eq, B_eq

    def _get_nearest_waypoint(self, x, y, prev_idx):
        """! Search the closest waypoint to the vehicle on the reference path
        @param x<float>: The x-coordinate of the vehicle
        @param y<float>: The y-coordinate of the vehicle
        @param prev_idx<int>: The previous index
        @return<int>: The index of the nearest waypoint
        """

        search_horizone_idx = prev_idx + self._predict_step

        dx = [x - ref_x for ref_x in self.trajectory.x[prev_idx:(prev_idx + search_horizone_idx), 0]]
        
        dy = [y - ref_y for ref_y in self.trajectory.x[prev_idx:(prev_idx + search_horizone_idx), 1]]

        d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

        min_d = min(d)
        nearest_idx = d.index(min_d) + prev_idx

        return nearest_idx

    def _plot_solution(self, state, input, index):
        """! Plot the solution
        @param state<list>: The state of the vehicle
        @param input<list>: The input of the vehicle
        @param index<int>: The index of the vehicle
        @return None
        """
        x_out = np.zeros((self._predict_step, self._nx))

        for i in range(self._predict_step):

            new_state = self.model.function(
                state, input, self.trajectory.sampling_time)

            state = new_state

            x_out[i, :] = new_state

        _, ax = plt.subplots(1, 1)

        ax.set_box_aspect(1)

        ax.plot(x_out[:, 0],
                x_out[:, 1], "r",
                label="Tracking performance")

        ax.plot(
            [path[0] for path in self.trajectory.x],
            [path[1] for path in self.trajectory.x],
            "--b", label="Trajectory")

        ax.legend()

        ax.set_xlabel("X [m]")

        ax.set_ylabel("Y [m]")

        plt.tight_layout()

        plt.show()

    # ==================================================================================================
    # STATIC METHODS
    # =================================================================================================
    @staticmethod
    def _convert_degrees_to_radians(degrees):
        """! Convert degrees to radians
        @param degrees<float>: The angle in degrees
        """
        radians = degrees * (math.pi / 180)

        return radians

    @staticmethod
    def _diag_mat(mat1, mat2):
        """! Create a block diagonal matrix
        @param mat1<np.array>: The first matrix
        @param mat2<np.array>: The second matrix
        """
        Z1 = np.zeros((mat1.shape[0], mat2.shape[1]))

        Z2 = np.zeros((mat2.shape[0], mat1.shape[1]))

        out = np.asarray(np.bmat([[mat1, Z1], [Z2, mat2]]))

        return out