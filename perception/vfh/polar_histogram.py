#!/usr/bin/env python3
##
# @file polar_histogram.py
#
# @brief Provide implementation of polar histogram for perception.
#
# @section author_doxygen_example Author(s)
# - Created by Tran Viet Thanh on 27/08/2024.
#
# Copyright (c) 2024 System Engineering Laboratory.  All rights reserved.


class PolarHistogram:
    """! Polar histogram

    The class provides implementation of polar histogram for perception.

    @note The polar histogram is used to represent the environment.
    A polar histogram means this, assuming bin_width=36
    (therefore num_bins = 360 / 36 = 10):

    index, corresponding_angle, histogram_angle
    0, 0, 123
    1, 36, 0
    2, 72, 30
    ...
    9, 324, 0

    (equation: index * bin_width = angle)

    However, we only keep index in a flat array for histograms, so we don't 
    natively get/_set by angle but instead translate to and from angle.
    """
    # ==================================================================================================
    # PUBLIC METHODS
    # ==================================================================================================

    def __init__(self, num_bins):
        """! Constructor
        @param num_bins<int>: The number of bins
        """
        self.num_bins = num_bins

        self.bin_width = 360/num_bins

        self._polar_histogram = [0] * num_bins

    def get_middle_angle_of_bin(self, bin_index):
        """! Returns the angle in the middle of the bin.
        @param bin_index<int>: The index of the bin
        @return float: The angle in the middle of the bin
        """
        bin_index = self._wrap(bin_index)

        return (bin_index + 0.5) * self.bin_width

    def add_certainty_to_bin_at_angle(self, angle, delta_certainty):
        """! Adds the passed value to the current value of the histogr1am grid.
        @param angle<float>: The angle
        @param delta_certainty<float>: The value to be added to the histogram
        """
        bin_index = self._get_bin_index_from_angle(angle)

        self._polar_histogram[bin_index] += delta_certainty

    def smooth_histogram(self, num_bins_to_consider):
        """! Smoothing function that smooths the values of the histogram using
        a moving average.
        @param num_bins_to_consider<int>: The number of bins to consider in
        each direction. In paper, this is l.
        @note This function is used to smooth the histogram values.
        """
        smoothed_histogram = [0] * self.num_bins

        for k_i in range(self.num_bins):
            list_of_bins = [(num_bins_to_consider - abs(k_i-l_i)) * self._get(
                l_i) for l_i in range(
                    k_i-num_bins_to_consider+1, k_i+num_bins_to_consider)]

            smoothed_histogram[k_i] = sum(list_of_bins)

            smoothed_histogram[k_i] /= (2*num_bins_to_consider+1)

        self._polar_histogram = smoothed_histogram

    def __str__(self):
        """! Returns the string representation of the histogram.
        @return str: The string representation of the histogram
        """
        string = 'index, angle, certainty\n'

        for i, certainty in enumerate(self._polar_histogram):
            string += str(i) + ' ' + str(i * self.bin_width) + \
                ' ' + str(certainty) + '\n'

        return string

    def reset(self):
        """! Resets the histogram to zero.
        """
        self._polar_histogram = [0] * self.num_bins

    # ==================================================================================================
    # PRIVATE METHODS
    # ==================================================================================================
    def _wrap(self, bin_index):
        """! Helper method for covering out of bounds bin_index.
        @param bin_index<int>: The index of the bin
        @return int: The wrapped index of the bin
        """
        while bin_index < 0:
            bin_index += self.num_bins

        while bin_index >= self.num_bins:
            bin_index -= self.num_bins

        return bin_index

    def _get(self, bin_index):
        """! Custom getter covering cases where bin_index is out of bounds.
        @param bin_index<int>: The index of the bin
        @return float: The value of the histogram for the specified bin
        """
        bin_index = self._wrap(bin_index)

        return self._polar_histogram[bin_index]

    def _set(self, bin_index, value):
        """! Custom setter covering cases where bin_index is out of bounds.
        @param bin_index<int>: The index of the bin
        @param value<float>: The value of the histogram for the specified bin
        """
        bin_index = self._wrap(bin_index)

        self._polar_histogram[bin_index] = value

    def _get_bin_index_from_angle(self, angle):
        """! Returns index 0 <= index < number_of_bins that corresponds to a.
        "_wrap" aaround histogram.
        @param angle<float>: The angle
        @return int: The index of the bin
        """
        while angle < 0:
            angle += 360

        while angle > 360:
            angle -= 360

        return int(angle // self.bin_width)
