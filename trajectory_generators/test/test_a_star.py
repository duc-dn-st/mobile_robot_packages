
# Standard library
import os
import sys

sys.path.append(os.path.join("..",".."))

# Internal library
from trajectory_generators.a_star import AStar
from environments.graph import Graph


if __name__ == "__main__":
    environment = Graph()

    trajectory_generator = AStar(environment)

    start = (1.0, 1.0)

    end = (5.0, 1.0)

    path, length = trajectory_generator.generate(start, end)

    print(path)
