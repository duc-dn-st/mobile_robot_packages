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
from scipy.linalg import block_diag
import matplotlib.pyplot as plt
import time

# External library
import casadi as cs

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



    """ Controller : Prediction Model
    - state values : 
    x = [x[0], x[1], x[2]] = [x_e, y_e, \theta_e]
    - input values : 
    u = [u[0], u[1]] = [v, \omega] (front steer angle, acceleration)

    """

    # ==================================================================================
    # Parameters
    nx = model.nx

    nu = model.nu
        
    starting_step = 0

    N = 100

    q_terminal_weight_val = [200, 200, 2]

    q_stage_weight_val = [10, 10, 0.1]

    r_stage_weight_val = [1, 1]

    # Construct nlp optimizer
    optimizer = cs.Opti()

    optimizer.solver('ipopt', {'print_time': False},
                    {'max_iter': 3000, 'acceptable_iter': 1000,
                    'print_level': 5, 'tol': 0.0001,
                    'acceptable_tol': 0.1, 'mu_init': 0.05,
                    'warm_start_init_point': 'yes'})

    # Set optimization variable
    u = optimizer.variable(nu, N)

    # reference trajectory
    time_stamp = trajectory.t[starting_step:starting_step + N + 1]
    state_ref = trajectory.x[starting_step:starting_step + N + 1]
    input_ref = trajectory.u[starting_step:starting_step + N]

    # Set optimizer parameters
    dt = optimizer.parameter(1)

    q_terminal_weight = optimizer.parameter(len(q_terminal_weight_val))
    q_stage_weight = optimizer.parameter(len(q_stage_weight_val))
    r_stage_weight = optimizer.parameter(len(r_stage_weight_val))
    
    initial_position = optimizer.parameter(nx)
    # final_position = optimizer.parameter(nx)

    # Set value to optimizer parameters
    optimizer.set_value(dt, trajectory.sampling_time)
    optimizer.set_value(q_stage_weight, q_stage_weight_val)
    optimizer.set_value(q_terminal_weight, q_terminal_weight_val)
    optimizer.set_value(r_stage_weight, r_stage_weight_val)
    optimizer.set_value(initial_position, state_ref[0, :])
    # optimizer.set_value(final_position, state_ref[1, :])

    optimizer.set_initial(u, np.transpose(input_ref))

    # Set up optimization problem
    x = cs.horzcat(initial_position)
    cost = 0

    final_position = state_ref

    for i in range(0, N-1):

        # optimizer.set_value(final_position, state_ref[i+1, :])

        state_error = x - final_position[i+1, :]

        # State cost
        cost += q_stage_weight[0] * state_error[0]**2 \
                + q_stage_weight[1] * state_error[1]**2 \
                + q_stage_weight[2] * state_error[2]**2 \
                + r_stage_weight[0] * u[0, i]**2 \
                + r_stage_weight[1] * u[1, i]**2     
        
        # Update state
        x = model.casadi_function(x, u[:, i], dt)

        optimizer.subject_to(u[0]**2 < model.velocity_max**2)

        optimizer.subject_to(u[1]**2 < model.angular_velocity_max**2)

    # Terminal cost
    # optimizer.set_value(final_position, state_ref[-1, :])
    
    state_error = x - final_position[-1, :]

    cost += q_terminal_weight[0] * state_error[0]**2 \
            + q_terminal_weight[1] * state_error[1]**2 \
            + q_terminal_weight[2] * state_error[2]**2

    optimizer.minimize(cost)

    start_time = time.time()

    solution = optimizer.solve() 

    run_time = time.time() - start_time

    # Print runtime
    print(f"Runtime: {run_time} seconds")

    # print(solution.value(u))

    """! Simulation
    """

    x_log = np.array([])

    x0 = state_ref[0, :]

    x_log = np.append(x_log, x0)

    for i in range(N):
            
        x0 = model.function(x0, solution.value(u)[:, i], trajectory.sampling_time)

        x_log = np.append(x_log, x0)

    x_log = x_log.reshape(-1, nx)

    # print(x_log.shape)

    _, ax = plt.subplots(1, 1)

    ax.set_box_aspect(1)

    ax.plot(x_log[:, 0],
            x_log[:, 1], "r",
            label="Tracking performance")

    ax.plot(
        [path[0] for path in trajectory.x[0:N, :]],
        [path[1] for path in trajectory.x[0:N, :]],
        "--b", label="Trajectory")

    ax.legend()

    ax.set_xlabel("X [m]")

    ax.set_ylabel("Y [m]")

    plt.tight_layout()

    _, (linear_ax, angular_ax) = plt.subplots(1, 2)

    number_of_input = len(optimizer.value(u)[0, :])

    linear_ax.plot(time_stamp[:number_of_input],
                    optimizer.value(u)[0, :])

    angular_ax.plot(time_stamp[:number_of_input],
                    optimizer.value(u)[1, :])

    linear_ax.set_xlabel("Time [s]")

    linear_ax.set_ylabel("Velocity [m/s]")

    angular_ax.set_xlabel("Time [s]")

    angular_ax.set_ylabel("Angular Velocity [rad/s]")

    plt.tight_layout()

    plt.show()
