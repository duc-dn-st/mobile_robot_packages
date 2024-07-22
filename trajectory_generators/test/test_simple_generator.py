
# Standard library
import os
import sys
import matplotlib.pyplot as plt

sys.path.append(os.path.join("..",".."))

# Internal library
from environments.graph import Graph
from trajectory_generators.simple_generator import SimpleGenerator


if __name__ == "__main__":
    environment = Graph()

    trajectory = SimpleGenerator(environment)

    trajectory.generate("eight_curve.csv", nx=3, nu=2)

    print(trajectory.x)

    print(trajectory.u)

    print(trajectory.t)
