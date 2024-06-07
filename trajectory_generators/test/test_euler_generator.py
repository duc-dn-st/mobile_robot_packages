
# Standard library
import os
import sys
import matplotlib.pyplot as plt

sys.path.append(os.path.join("..",".."))

# Internal library
from trajectory_generators.euler_generator import EulerGenerator
from environments.graph import Graph


if __name__ == "__main__":
    environment = Graph()

    trajectory_generator = EulerGenerator(environment)

    start = (1.0, 1.0)

    end = (8.0, 8.0)

    position, euler_paths = trajectory_generator.generator(start, end)

    figure, ax = plt.subplots()

    ax.set_box_aspect(1)

    ax.plot([path[0] for path in trajectory_generator.a_star_paths], [path[1] for path in trajectory_generator.a_star_paths])

    ax.plot([path[0] for path in euler_paths], [path[1] for path in euler_paths])

    ax.plot([path[0] for path in position], [path[1] for path in position])

    plt.show()

