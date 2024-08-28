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

    def __init__(self, map_name, active_region_dimension=(8, 8), resolution=1,
                 num_bins=36, a=200, b=1, num_bins_to_consider=5,
                 s_max=15, valley_threshold=200):
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

    def set_target_location(self, target_discrete_location):
        """! Set the target discrete location
        @param target_discrete_location<tuple>: The location of the target
        """
        self.histogram_grid.set_target_discrete_location(
            target_discrete_location)

    def set_robot_location(self, robot_location):
        """! Set the robot location
        @param robot_location<tuple>: The location of the robot
        """
        self._generate_histogram(robot_location)

    def get_best_angle(self, continuous_angle):
        """! Get the best angle
        @param continuous_angle<float>: The angle between the robot and
        the target
        @return float: The best angle
        """
        sectors = self._get_sectors()

        if len(sectors) == 0:
            least_likely_bin = sorted(
                range(len(self.polar_histogram._polar_histogram)),
                key=lambda k: self.polar_histogram._polar_histogram[k])[0]

            middle_angle = self.polar_histogram.get_middle_angle_of_bin(
                least_likely_bin)

            return middle_angle

        if len(sectors) == 1:
            return continuous_angle / math.pi * 180

        angles = []

        for sector in sectors:
            angle = self._calculate_angle_by_sector(sector, continuous_angle)

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
        self.polar_histogram.reset()

        min_x, min_y, max_x, max_y = self.histogram_grid.get_active_region(
            robot_location)

        histogram_grid = self.histogram_grid

        polar_histogram = self.polar_histogram

        for x in range(min_x, max_x):
            for y in range(min_y, max_y):
                node_considered = (x, y)

                certainty = histogram_grid.get_certainty_at_discrete_point(
                    node_considered)

                distance = histogram_grid.get_distance_between_discrete_points(
                    node_considered, robot_location)

                delta_certainty = (certainty ** 2) * \
                    (self.a - self.b * distance)

                angle = histogram_grid.get_angle_between_discrete_points(
                    robot_location, node_considered)

                if delta_certainty != 0:
                    polar_histogram.add_certainty_to_bin_at_angle(
                        angle, delta_certainty)

        polar_histogram.smooth_histogram(self.num_bins_to_consider)

    def _get_filtered_polar_histogram(self):
        """! Get the filtered polar histogram
        @return list: The filtered polar histogram
        """
        return [bin_index for bin_index, certainty in enumerate(
            self.polar_histogram._polar_histogram
        ) if certainty < self.valley_threshold]

    def _get_sectors_from_filtered_polar_histogram(
            self, filtered_polar_histogram):
        """! Get the sectors from the filtered polar histogram
        @param filtered_polar_histogram<list>: The filtered polar histogram
        @return list: The sectors from the filtered polar histogram
        """
        return [list(map(itemgetter(1), g)) for _, g in groupby(enumerate(
            filtered_polar_histogram), lambda ix: ix[0] - ix[1])]

    def _get_sectors(self):
        """! Get the sectors
        @return list: The sectors
        """
        filtered_polar_histogram = self._get_filtered_polar_histogram()

        sectors = self._get_sectors_from_filtered_polar_histogram(
            filtered_polar_histogram)

        return sectors

    def _calculate_angle_by_sector(self, sector, continuous_angle):
        """! Calculate the angle by sector
        @param sector<list>: The sector
        @param continuous_angle<float>: The angle between the robot and
        @return float: The angle
        @note The angle is calculated by the sector, if the sector is wide,
        the angle is calculated by the closest bin to the target direction,
        otherwise, the angle is calculated by the middle of the sector.
        The wideness of the sector by absolutes difference between the first
        and last bin of the sector.
        """
        if len(sector) > self.s_max:
            if abs(sector[0] - continuous_angle) > abs(
                    sector[-1] - continuous_angle):
                k_n = 0

                k_f = k_n + self.s_max - 1

            else:
                k_n = len(sector) - 1

                k_f = k_n - self.s_max + 1

        else:
            k_n = sector[0]

            k_f = sector[-1]

        angle = (self.polar_histogram.get_middle_angle_of_bin(
            k_n) + self.polar_histogram.get_middle_angle_of_bin(k_f)) / 2

        return angle
