# Standard library
import os
import sys
import math

# Internal library
sys.path.append(os.path.join("..", "..", ".."))
from perception.vfh.polar_histogram import PolarHistogram  # noqa
from perception.vfh.histogram_grid import HistogramGrid  # noqa
from perception.vfh.vector_field_histogram import VectorFieldHistogram  # noqa


def update_location(path_planner, angle, velocity, location):
    # angle_radian = angle * math.pi/180
    velocity_x, velocity_y = velocity

    old_x, old_y = location
    location = (old_x + velocity_x, old_y + velocity_y)

    # Why does path_planner need discrete location?
    discrete_location = path_planner.\
        histogram_grid.continuous_point_to_discrete_point(
            location)
    print("robot: discrete_location =", discrete_location)
    path_planner.set_robot_location(discrete_location)

    return location


def update_angle(path_planner, location, target_location):
    continuous_displacement = (
        target_location[0] - location[0], target_location[1] - location[1])
    continuous_robot_to_target_angle = math.atan2(
        continuous_displacement[1], continuous_displacement[0])
    angle = path_planner.get_best_angle(continuous_robot_to_target_angle)
    continuous_robot_to_target_angle = continuous_robot_to_target_angle

    return angle


def update_velocity(angle, speed):
    angle_radian = angle * math.pi/180
    # old_v_x, old_v_y = velocity
    velocity = (speed * math.cos(angle_radian), speed * math.sin(angle_radian))

    return velocity


def step(path_planner, location, target_location, angle):
    # path_planner.print_histogram()
    # angle: Null (or optionally, t-1) => t
    angle = update_angle(path_planner, location, target_location)
    # set_speed() # speed: Null (or optionally, t-1) => t
    print("robot: step: best angle =", angle)
    velocity = update_velocity(angle, 1.0)
    print("robot: step: velocity =", velocity)
    location = update_location(
        path_planner, angle, velocity, location)  # position: t => t+1
    print("robot: step: location =", location)
    return angle, velocity, location


if __name__ == "__main__":
    position = (0.0, 0.0)

    active_region_dimension = (8, 8)

    resolution = 1

    map_fname = 'map.txt'

    histogram_grid = HistogramGrid.from_map(
        map_fname, active_region_dimension, resolution)

    num_bins = 36

    target_location = (50, 50)

    polar_histogram = PolarHistogram(num_bins)

    path_planner = VectorFieldHistogram(
        histogram_grid, polar_histogram, position, target_location)

    num_steps = 20

    continuous_displacement = (
        target_location[0] - position[0], target_location[1] - position[1])

    continuous_robot_to_target_angle = math.atan2(
        continuous_displacement[1], continuous_displacement[0])

    angle = path_planner.get_best_angle(continuous_robot_to_target_angle)

    location = position

    for index in range(num_steps):

        angle, velocity, location = step(
            path_planner, location, target_location, angle)

        print("vfh angle: ", angle)
