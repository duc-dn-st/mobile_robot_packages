
# Standard library
import os
import sys
from matplotlib import pyplot as plt

# Internal library
sys.path.append(os.path.join("..", ".."))
from environments.graph import Graph # noqa
from trajectory_generators.simple_generator import SimpleGenerator # noqa


if __name__ == "__main__":
    environment = Graph()

    trajectory = SimpleGenerator(environment)

    trajectory.generate("global_trajectory.csv", nx=3, nu=2)

    initial_paths = [(0.0, 0.0), (5.0, 0.0), (5.0, 5.0)]

    print(trajectory.x)

    print(trajectory.u)

    print(trajectory.t)

    figure, ax = plt.subplots()

    ax.plot(trajectory.x[:, 0], trajectory.x[:, 1])

    ax.plot([path[0] for path in initial_paths],
            [path[1] for path in initial_paths])

    plt.show()
