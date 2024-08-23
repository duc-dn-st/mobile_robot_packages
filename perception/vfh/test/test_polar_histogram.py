
# Standard library
import os
import sys

sys.path.append(os.path.join("..","..",".."))


# Internal library
from perception.vfh.polar_histogram import PolarHistogram


if __name__ == "__main__":

    num_bins = 36 # each bin is 360/num_bins degrees
    
    target_location = (50, 50)

    polar_histogram = PolarHistogram(num_bins)

    print("angle certainty: ", polar_histogram.get_angle_certainty())