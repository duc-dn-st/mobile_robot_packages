#!/usr/bin/env python3
##
# @file vector_field_histogram.py
#
# @brief Provide implementation of vector field histogram for perception.
#
# @section author_doxygen_example Author(s)
# - Created by Tran Viet Thanh on 27/08/2024.
#
# Copyright (c) 2024 System Engineering Laboratory.  All rights reserved.

# Standard library
import math
import numpy as np
from itertools import groupby
from operator import itemgetter

# Internal library
from perception.vfh.polar_histogram import PolarHistogram  # noqa
from perception.vfh.histogram_grid import HistogramGrid  # noqa


class VectorFieldHistogram:
    """! Vector Field Histogram (VFH)

    The class provides implementation of Vector Field Histogram (VFH) for
    perception.
    """
    # ==================================================================================================
    # PUBLIC METHODS
    # ==================================================================================================

    def __init__(self, map_name, active_region_dimension=(8, 8),
                 resolution=1, num_bins=36, a=200, b=1,
                 num_bins_to_consider=5, s_max=5, valley_threshold=50):
        """! Constructor
        @param map_name<str>: The name of the map
        @param active_region_dimension<tuple>: The dimension of the active
        region
        @param resolution<float>: The resolution of the grid
        @param num_bins<int>: The number of bins
        @param a<float>: The parameter a
        @param b<float>: The parameter b
        @param num_bins_to_consider<int>: The number of bins to consider
        @param s_max<int>: The maximum number of bins in a sector
        @param valley_threshold<float>: The threshold of the valley
        """
        self.polar_histogram = PolarHistogram(num_bins)

        self.histogram_grid = HistogramGrid.from_map(
            map_name, active_region_dimension, resolution)

        self.a = a

        self.b = b

        self.num_bins_to_consider = num_bins_to_consider

        self.s_max = s_max

        self.valley_threshold = valley_threshold

    def set_robot_location(self, robot_location):
        """! Set the robot location
        @param robot_location<tuple>: The location of the robot
        """
        self.robot_location = robot_location

        self._generate_histogram(robot_location)

    def get_best_angle(self, target_location):
        """! Get the best angle
        @param target_location<tuple>: The location of the target
        @return float: The best angle
        """
        target_sector, continuous_angle = self._calculate_sector_to_target(
            target_location)

        sectors = self._get_sectors()

        if len(sectors) == 0:
            least_likely_bin = sorted(
                range(len(self.polar_histogram._polar_histogram)),
                key=lambda k: self.polar_histogram._polar_histogram[k])[0]

            middle_angle = self.polar_histogram.get_middle_angle_of_bin(
                least_likely_bin)

            return middle_angle

        if len(sectors) == 1:
            return self._calculate_angle_by_sector(
                sectors[0], target_sector)

        angles = []

        for sector in sectors:
            angle = self._calculate_angle_by_sector(
                sector, target_sector)

            angles.append(angle)

        distances = [(angle, abs(continuous_angle - angle))
                     for angle in angles]

        smallest_distances = sorted(distances, key=itemgetter(1))

        best_angle = smallest_distances[0][0]

        return best_angle

    # ==================================================================================================
    # PRIVATE METHODS
    # ==================================================================================================
    def _generate_histogram(self, robot_location):
        """! Generate the histogram
        @param robot_location<tuple>: The location of the robot
        """
        discrete_location = self.histogram_grid.\
            continuous_point_to_discrete_point(robot_location)

        self.polar_histogram.reset()

        min_x, min_y, max_x, max_y = self.histogram_grid.get_active_region(
            discrete_location)

        histogram_grid = self.histogram_grid

        polar_histogram = self.polar_histogram

        for x in range(min_x, max_x):
            for y in range(min_y, max_y):
                node_considered = (x, y)

                certainty = histogram_grid.get_certainty_at_discrete_point(
                    node_considered)

                distance = histogram_grid.get_distance_between_discrete_points(
                    node_considered, discrete_location)

                delta_certainty = (certainty ** 2) * \
                    (self.a - self.b * distance)

                angle = histogram_grid.get_angle_between_discrete_points(
                    discrete_location, node_considered)

                if delta_certainty != 0:
                    polar_histogram.add_certainty_to_bin_at_angle(
                        angle, delta_certainty)

        polar_histogram.smooth_histogram(self.num_bins_to_consider)

    def _get_filtered_polar_histogram(self):
        """! Get the filtered polar histogram
        @return list: The filtered polar histogram
        """
        return [(bin_index, certainty) for bin_index, certainty in enumerate(
            self.polar_histogram._polar_histogram
        ) if certainty < self.valley_threshold]

    def _group_sectors(self, polar_histogram):
        """! Group the sectors
        @param polar_histogram<list>: The polar histogram
        @return list: The sectors
        """
        return [list(group) for _, group in groupby(
            polar_histogram, key=lambda x: x[0] - polar_histogram[0][0])]

    def _get_sectors_from_filtered_polar_histogram(
            self, filtered_polar_histogram):
        """! Get the sectors from the filtered polar histogram
        @param filtered_polar_histogram<list>: The filtered polar histogram
        @return list: The sectors from the filtered polar histogram
        """
        groups = [list(group) for _, group in groupby(
            [(bin_index, certainty) for bin_index, certainty in
             filtered_polar_histogram],
            key=lambda x, c=iter(range(1000)): next(c) - x[0]
        )]

        sectors = [[bin_index for bin_index, _ in group]
                   for group in groups]

        return sectors

    def _get_sectors(self):
        """! Get the sectors
        @return list: The sectors
        """
        filtered_polar_histogram = self._get_filtered_polar_histogram()

        sectors = self._get_sectors_from_filtered_polar_histogram(
            filtered_polar_histogram)

        return sectors

    def _calculate_sector_to_target(self, target_location):
        """! Calculate the angle to the target
        @param target_location<tuple>: The location of the target
        @return float: The angle
        """
        position = self.robot_location

        continuous_displacement = (
            target_location[0] - position[0], target_location[1] - position[1])

        continuous_robot_to_target_angle = math.atan2(
            continuous_displacement[1], continuous_displacement[0])

        continuous_robot_to_target_angle = np.arctan2(
            np.sin(continuous_robot_to_target_angle),
            np.cos(continuous_robot_to_target_angle)
        )

        bin_index = self.polar_histogram._get_bin_index_from_angle(
            np.rad2deg(continuous_robot_to_target_angle))

        return bin_index, continuous_robot_to_target_angle

    def _calculate_angle_by_sector(self, sector, target_sector):
        """! Calculate the angle by sector
        @param sector<list>: The sector
        @param target_sector<list>: The target sector
        @return float: The angle
        @note The angle is calculated by the sector, if the sector is wide,
        the angle is calculated by the closest bin to the target direction,
        otherwise, the angle is calculated by the middle of the sector.
        The wideness of the sector by absolutes difference between the first
        and last bin of the sector.
        """
        if target_sector in sector:
            return self.polar_histogram.get_middle_angle_of_bin(target_sector)

        if len(sector) > self.s_max:
            k_n = min(
                sector, key=lambda k: abs(k - target_sector))

            k_f = k_n + self.s_max if k_n > target_sector else k_n - self.s_max

            k_f = self.polar_histogram._wrap(k_f)
        else:
            k_n = sector[0]

            k_f = sector[-1]

        angle = (self.polar_histogram.get_middle_angle_of_bin(
            k_n) + self.polar_histogram.get_middle_angle_of_bin(k_f)) / 2

        return angle
