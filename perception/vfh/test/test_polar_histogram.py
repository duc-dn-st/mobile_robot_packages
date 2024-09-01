
# Standard library
import os
import sys

# Internal library
sys.path.append(os.path.join("..", "..", ".."))
from perception.vfh.polar_histogram import PolarHistogram  # noqa


if __name__ == "__main__":

    num_bins = 36  # each bin is 360/num_bins degrees

    target_location = (50, 50)

    polar_histogram = PolarHistogram(num_bins)
