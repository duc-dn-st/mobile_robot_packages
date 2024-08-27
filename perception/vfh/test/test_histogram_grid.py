
# Standard library
import os
import sys

sys.path.append(os.path.join("..", "..", ".."))


# Internal library
from perception.vfh.histogram_grid import HistogramGrid  # noqa


if __name__ == "__main__":

    position = (0.0, 0.0)

    active_region_dimension = (8, 8)

    resolution = 1

    map_fname = 'map.txt'

    histogram_grid = HistogramGrid.from_map(
        map_fname, active_region_dimension, resolution, position)

    print("active region: ", histogram_grid.get_active_region((1.0, 1.0)))

    print("obstacles: ", histogram_grid.get_obstacles())
  
