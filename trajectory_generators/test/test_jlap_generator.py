
# Standard library
import os
import sys


# Internal library
sys.path.append(os.path.join("..", ".."))
from trajectory_generators.jlap_generator import JlapGenerator # noqa


if __name__ == "__main__":

    trajectory_generator = JlapGenerator()

    initial_paths = [(1.0, 1.0), (3.0, 7.0)]

    position = trajectory_generator.generate(initial_paths)
