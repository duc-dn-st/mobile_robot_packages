#!/usr/bin/env python3
##
# @file histogram_grid.py
#
# @brief Provide implementation of histogram grid for perception.
#
# @section author_doxygen_example Author(s)
# - Created by Tran Viet Thanh on 27/08/2024.
#
# Copyright (c) 2024 System Engineering Laboratory.  All rights reserved.

# Standard library
import csv
import math
import numpy as np
from operator import sub


class HistogramGrid:
    """! Histogram grid

    The class provides implementation of histogram grid for perception.
    @note The histogram grid is used to represent the environment.
    """
    # ==================================================================================================
    # PUBLIC METHODS
    # ==================================================================================================

    def __init__(self, dimension, resolution, active_region_dimension):
        """! Constructor
        @param dimension<tuple>: The dimension of the grid
        @param resolution<float>: The resolution of the grid
        @param active_region_dimension<tuple>: The dimension of the active
        region
        """
        self.dimension = dimension

        self.resolution = resolution

        ncols, nrows = dimension

        self.histogram_grid = [[0] * ncols for _ in range(nrows)]

        self.active_region_dimension = active_region_dimension

    @classmethod
    def from_map(cls, map_fname, active_region_dimension, resolution):
        """! Create histogram grid from map
        @param map_fname<str>: The file name of the map
        @param active_region_dimension<tuple>: The dimension of the active
        region
        @param resolution<float>: The resolution of the grid
        @return HistogramGrid: The histogram grid
        """
        with open(map_fname, 'r') as f:
            reader = csv.reader(f, delimiter=" ")

            lines = list(reader)

        lines = list(map(lambda line: list(map(int, line)), lines))

        dimension = (len(lines[0]), len(lines))

        hg = cls(dimension, resolution, active_region_dimension)

        hg.histogram_grid = lines[::-1]

        return hg

    def continuous_point_to_discrete_point(self, continuous_point):
        """! Calculates in which node an object exists based on the continuous
        (exact) coordinates
        @param continuous_point<tuple>: The continuous point
        @return tuple: The discrete point
        """
        continuous_x, continuous_y = continuous_point

        discrete_x = continuous_x // self.resolution

        discrete_y = continuous_y // self.resolution

        return discrete_x, discrete_y

    def get_certainty_at_discrete_point(self, discrete_point):
        """! Returns the certainty of an object being present at the given node
        @param discrete_point<tuple>: The discrete point
        @return int: The certainty
        """
        discrete_x, discrete_y = discrete_point

        return self.histogram_grid[discrete_y][discrete_x]

    def get_distance_between_discrete_points(
            self, discrete_start, discrete_end):
        """! Returns scalar distance between two discretePoints (pos1 & pos2)
        on the histogram grid
        @param discrete_start<tuple>: The start point
        @param discrete_end<tuple>: The end point
        @return float: The distance
        """
        discrete_displacement = self._get_discrete_displacement(
            discrete_start, discrete_end)

        continuous_displacement = tuple(
            self.resolution * axis for axis in discrete_displacement)

        continuous_distance = math.sqrt(
            sum(axis**2 for axis in continuous_displacement))

        return continuous_distance

    @staticmethod
    def get_angle_between_discrete_points(discrete_start, discrete_end):
        """! Returns the angle between the line between pos2 and posRef and
        the horizontal along positive i direction.
        @param discrete_start<tuple>: The start point
        @param discrete_end<tuple>: The end point
        @return float: The angle
        """
        angle = math.atan2(
            discrete_end[1] - discrete_start[1],
            discrete_end[0] - discrete_start[0]
        )

        angle = np.arctan2(np.sin(angle), np.cos(angle))

        return np.rad2deg(angle)

    def get_active_region(self, robot_location):
        """! Returns the active region of the histogram grid
        @param robot_location<tuple>: The location of the robot
        @return tuple: The active region
        """
        x, y = robot_location

        size_x, size_y = self.active_region_dimension

        x_max, y_max = self.dimension

        active_region_min_x = max(0, x - size_x / 2)

        active_region_min_y = max(0, y - size_y / 2)

        active_region_max_x = min(x_max, x + size_x / 2)

        active_region_max_y = min(y_max, y + size_y / 2)

        active_region = (int(round(active_region_min_x)),
                         int(round(active_region_min_y)),
                         int(round(active_region_max_x)),
                         int(round(active_region_max_y)))

        return active_region

    def get_obstacles(self):
        """! Returns the obstacles
        @return tuple: The obstacles
        """
        obstacles = []

        for row_idx, row in enumerate(self.histogram_grid):
            for col_idx, cell in enumerate(row):
                if cell == 0:
                    continue

                obstacles.append((row_idx, col_idx))

        return obstacles

    # ==================================================================================================
    # PRIVATE METHODS
    # ==================================================================================================
    @staticmethod
    def _get_discrete_displacement(discrete_start, discrete_end):
        """! Returns the displacement between two discrete points
        @param discrete_start<tuple>: The start point
        @param discrete_end<tuple>: The end point
        @return tuple: The displacement
        """
        return tuple(map(sub, discrete_start, discrete_end))
