import numpy as np
import matplotlib.pyplot as plt

# Define the helper function
def jlap_acceleration_phase_displacement_calculator(vel_start, acc_start, vel_end, acc_end):
    # Mock implementation of the acceleration phase displacement calculator.
    jerk_time = 1.0
    accel_time = (vel_end - vel_start) / 2
    const_vel_time = 1.0
    displacement = (vel_start + vel_end) * accel_time / 2
    
    return np.array([jerk_time, accel_time, const_vel_time, displacement])

# Define the main function
def point_to_point_motion_jlap_solver(n_line, n_corner, n_point, way_points, pos_start_corner, pos_end_corner, path_vel_current, path_vel_corner, PATH_VEL_LIM, PATH_VEL_STEP):
    T_jlap = np.zeros((n_line, 7))
    
    pos_start = np.zeros((n_corner + 1, 2))
    pos_end = np.zeros((n_corner + 1, 2))
    
    pos_start[:n_corner] = way_points[0, :]
    pos_start[n_corner] = pos_end_corner
    pos_end[:n_corner] = pos_start_corner
    pos_end[n_corner] = way_points[n_point - 1, :]

    path_vel_start = np.zeros(n_corner + 1)
    path_vel_end = np.zeros(n_corner + 1)
    
    path_vel_start[0] = path_vel_current
    path_vel_start[1:] = path_vel_corner
    path_vel_end[:-1] = path_vel_corner
    path_vel_end[-1] = 0

    for i in range(n_line):
        disp_vector = pos_end[i] - pos_start[i]
        line_length = np.linalg.norm(disp_vector)  # [m] Line length

        temp = np.array([path_vel_start[i], path_vel_end[i]])
        path_vel_max = 1.1 * np.max(temp)
        
        if path_vel_max > PATH_VEL_LIM:
            path_vel_max = PATH_VEL_LIM

        path_acc_start = 0  # [m/s^2] Acceleration at the initial acceleration phase
        path_acc_end = 0    # [m/s^2] Acceleration at the end of the acceleration phase

        data_a = jlap_acceleration_phase_displacement_calculator(path_vel_start[i], path_acc_start, path_vel_max, path_acc_end)
        data_d = jlap_acceleration_phase_displacement_calculator(path_vel_max, path_acc_start, path_vel_end[i], path_acc_end)

        displacement = data_a[3] + data_d[3]

        if (displacement > line_length):
            while displacement > line_length and path_vel_max > path_vel_end[i]:
                temp = np.array([path_vel_max - PATH_VEL_STEP, path_vel_end[i]])
                path_vel_max = np.max(temp)
                data_a = jlap_acceleration_phase_displacement_calculator(path_vel_start[i], path_acc_start, path_vel_max, path_acc_end)
                data_d = jlap_acceleration_phase_displacement_calculator(path_vel_max, path_acc_start, path_vel_end[i], path_acc_end)
                displacement = data_a[3] + data_d[3]
        else:
            while displacement < line_length and path_vel_max < PATH_VEL_LIM:
                temp = np.array([path_vel_max + PATH_VEL_STEP, PATH_VEL_LIM])
                path_vel_max = np.min(temp)
                data_a = jlap_acceleration_phase_displacement_calculator(path_vel_start[i], path_acc_start, path_vel_max, path_acc_end)
                data_d = jlap_acceleration_phase_displacement_calculator(path_vel_max, path_acc_start, path_vel_end[i], path_acc_end)
                displacement = data_a[3] + data_d[3]

        data_a = jlap_acceleration_phase_displacement_calculator(path_vel_start[i], path_acc_start, path_vel_max, path_acc_end)
        data_d = jlap_acceleration_phase_displacement_calculator(path_vel_max, path_acc_start, path_vel_end[i], path_acc_end)
        displacement = data_a[3] + data_d[3]

        cruise_dist = line_length - displacement
        temp = np.array([cruise_dist, 0])
        temp = temp / path_vel_max
        Tc = np.max(temp)  # [m/s] Cruise velocity time interval

        T_jlap[i, :] = np.array([data_a[0], data_a[1], data_a[2], Tc, data_d[0], data_d[1], data_d[2]])

    return T_jlap

# Parameters
n_line = 5
n_corner = 4
n_point = 6
way_points = np.random.rand(n_point, 2)
pos_start_corner = np.random.rand(n_corner, 2)
pos_end_corner = np.random.rand(2)
path_vel_current = 1.0
path_vel_corner = np.random.rand(n_corner)
PATH_VEL_LIM = 2.0
PATH_VEL_STEP = 0.1

# Compute JLAP time intervals
T_jlap = point_to_point_motion_jlap_solver(n_line, n_corner, n_point, way_points, pos_start_corner, pos_end_corner, path_vel_current, path_vel_corner, PATH_VEL_LIM, PATH_VEL_STEP)

# Plotting the results
plt.figure(figsize=(12, 6))
time_intervals = ['Jerk Time (a)', 'Accel Time (a)', 'Const Vel Time (a)', 'Cruise Time', 'Jerk Time (d)', 'Accel Time (d)', 'Const Vel Time (d)']
for i in range(7):
    plt.plot(range(n_line), T_jlap[:, i], label=time_intervals[i])

plt.xlabel('Line Segment')
plt.ylabel('Time [s]')
plt.title('JLAP Time Intervals for Each Line Segment')
plt.legend()
plt.grid(True)
plt.show()