#!/usr/bin/env python3
##
# @file dwa_with_eight_curve.py
#
# @brief Provide example for DWA tracking eight curve trajectory.
#
# @section author_doxygen_example Author(s)
# - Created by Dinh Ngoc DUc on 27/08/2024.

# Standard library
import sys
import math
import numpy as np
import cvxpy as cp
from scipy.linalg import block_diag
import matplotlib.pyplot as plt


# Internal library
sys.path.append('..')
from visualizers.plotter import Plotter  # noqa
from controllers.linear_mpc import LinearModelPredictiveControl  # noqa
from simulators.time_stepping import TimeStepping  # noqa
from models.differential_drive import DifferentialDrive  # noqa
from trajectory_generators.simple_generator import SimpleGenerator  # noqa

if __name__ == "__main__":

    """! Model
    """
    wheel_base = 0.53

    model = DifferentialDrive(wheel_base)

    """! Trajectory
    """
    trajectory = SimpleGenerator(model)

    trajectory.generate("ref_eight_curve.csv", nx=3, nu=2,
                        is_derivative=False)
    
    t_start = 0
    t_max = 60
    dt = trajectory.sampling_time
    

    """! Controller
    """

    x_0 = [0.0, 0.1, 0.0]

    # Set up optimization problem
    # Parameters
    nx = 3
    nu = 2
    predict_step = 100
    
    # optimization variable
    z = cp.Variable(nx * (predict_step + 1) + nu * predict_step)

    # Stage cost
    Q = np.diag([100.0, 100.0, 0.01])

    R = np.diag([0.001, 0.001])

    Q_stacked = block_diag(*([Q] * (predict_step+1)))

    R_stacked = block_diag(*([R] * predict_step))

    H = block_diag(Q_stacked, R_stacked)
    
    # Equality constraints
    
    A_eq = np.zeros((0, nx * (predict_step+1) + nu * predict_step))  

    B_eq = np.zeros((nx * (predict_step), 1))

    current_idx = 50

    state_ref = trajectory.x[current_idx:current_idx + predict_step]

    input_ref = trajectory.u[current_idx:current_idx + predict_step]

    for i in range(predict_step): 
    
        A_d, B_d = model.get_state_space_matrices(state_ref[i], input_ref[i], trajectory.sampling_time)

        A_temp = np.hstack((np.zeros((nx, i*nx)), A_d, -np.identity(nx), np.zeros((nx, nx*(predict_step-i-1)))))

        B_temp = np.hstack((i*np.zeros((nx, i*nu)), B_d, np.zeros((nx, nu*(predict_step-i-1)))))

        A_eq_temp = np.hstack((A_temp, B_temp))    

        A_eq = np.vstack((A_eq, A_eq_temp))

    # Inequality constraints

    G = np.matrix([[1, 0, -1, 0],
                    [-1, 0, 1, 0],
                    [0, 1, 0, -1],
                    [0, -1, 0, 1]])
    
    h = np.matrix([[model.velocity_max],
                [model.velocity_max],
                [model.angular_velocity_max],
                [model.angular_velocity_max]])

    A_in = np.zeros((0, (predict_step+1)*nx + predict_step*nu))

    B_in = np.zeros((0, 1))

    for i in range(predict_step-1):
            
        A_temp = np.hstack((np.zeros([G.shape[0] , nx*(predict_step+1)]), np.zeros([G.shape[0],i*nu]),  G, np.zeros([G.shape[0], predict_step*nu - G.shape[1] - i*nu])))

        A_in = np.vstack((A_in, A_temp))

        B_in = np.vstack((B_in, h))
        
    # Solve the problem
    objective = cp.Minimize(0.5 * cp.quad_form(z, H))

    constraints = [(A_eq @ z)[:, np.newaxis] == B_eq, (A_in @ z)[:, np.newaxis]  <= B_in]

    constraints += [z[0:nx] == x_0]

    prob = cp.Problem(objective, constraints)

    result = prob.solve()

    # Extract the solution

    state_sol = z.value[0:nx*(predict_step+1)].reshape((predict_step + 1, nx))

    input_sol = z.value[nx*(predict_step+1):].reshape((predict_step, nu))

    state = np.zeros((predict_step, nx))

    # Plot the result
    for i in range(predict_step):

        input = trajectory.u[i] + input_sol[i]

        new_state = model.function(x_0, input, dt)

        x_0 = new_state

        state[i,:] = new_state

    _, ax = plt.subplots(1, 1)

    ax.set_box_aspect(1)

    ax.plot(state[:, 0],
            state[:, 1], "r",
            label="Tracking performance")

    ax.plot(
        [path[0] for path in trajectory.x[0:predict_step, :]],
        [path[1] for path in trajectory.x[0:predict_step, :]],
        "--b", label="Trajectory")

    ax.legend()

    ax.set_xlabel("X [m]")

    ax.set_ylabel("Y [m]")

    plt.tight_layout()

    plt.show()




