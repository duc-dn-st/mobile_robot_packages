
# Standard library
import os
import sys
import matplotlib.pyplot as plt

sys.path.append(os.path.join("..",".."))

# Internal library
from trajectory_generators.jlap_generator import JlapGenerator
from environments.graph import Graph


if __name__ == "__main__":
    environment = Graph()

    trajectory_generator = JlapGenerator(environment)

    start = (1.0, 1.0)

    end = (8.0, 8.0)

    position, euler_paths = trajectory_generator.generator(start, end)


