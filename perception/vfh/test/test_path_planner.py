# Standard library
import os
import sys
import math
import csv
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle

# Internal library
sys.path.append(os.path.join("..", "..", ".."))
from perception.vfh.vector_field_histogram import VectorFieldHistogram  # noqa


def update_location(path_planner, angle, velocity, location):
    # angle_radian = angle * math.pi/180
    velocity_x, velocity_y = velocity

    old_x, old_y = location
    location = (old_x + velocity_x, old_y + velocity_y)

    print("robot: location =", location)

    path_planner.set_robot_location(location)

    return location


def update_angle(path_planner, target_location):
    """! Get the best angle
    @param location<tuple>: The location of the robot
    @param target_location<tuple>: The target location
    @return<float>: The best angle
    """
    angle = path_planner.get_best_angle(target_location)

    print("robot: update_angle: best angle =", angle)

    return angle


def update_velocity(angle, speed):
    angle_radian = angle * math.pi/180
    # old_v_x, old_v_y = velocity
    velocity = (speed * math.cos(angle_radian), speed * math.sin(angle_radian))

    return velocity


def step(path_planner, location, target_location, angle):
    # path_planner.print_histogram()
    # angle: Null (or optionally, t-1) => t
    angle = update_angle(path_planner, target_location)
    # set_speed() # speed: Null (or optionally, t-1) => t
    print("robot: step: best angle =", angle)
    velocity = update_velocity(angle, 1)
    print("robot: step: velocity =", velocity)
    location = update_location(
        path_planner, angle, velocity, location)  # position: t => t+1
    print("robot: step: location =", location)
    return angle, velocity, location


def _plot_obstacles(ax):
    """! Plot the obstacles.
    @param ax The axis object.
    """
    resolution = 1

    with open("map.txt", 'r') as f:
        reader = csv.reader(f, delimiter=" ")

        lines = list(reader)

    lines = list(map(lambda line: list(map(int, line)), lines))

    array = np.array(lines)

    for row in range(array.shape[0]):
        for column in range(array.shape[1]):
            if array[row, column] != 1:
                continue

            x = column * resolution

            y = (array.shape[0] - row - 1) * resolution

            rect = Rectangle((x, y), resolution, resolution, facecolor='black')

            ax.add_patch(rect)


if __name__ == "__main__":
    position = (40, 0.0)

    target_location = (0.0, 50)

    path_planner = VectorFieldHistogram("map.txt")

    path_planner.set_robot_location(position)

    num_steps = 100

    angle = path_planner.get_best_angle(target_location)

    location = position

    angles = []

    locations = [position]

    for index in range(num_steps):

        angle, velocity, location = step(
            path_planner, location, target_location, angle)

        if math.sqrt((location[0] - target_location[0]) ** 2 +
                     (location[1] - target_location[1]) ** 2) < 0.5:
            break

        locations.append(location)

        angles.append(angle)

    locations = np.array(locations)

    _, ax = plt.subplots()
    
    ax.plot(locations[:, 0], locations[:, 1], 'ro-')

    _, ax_angle = plt.subplots()

    ax_angle.plot(angles, 'r')

    _plot_obstacles(ax)

    plt.show()
