# Standard library
import os
import sys
from matplotlib import pyplot as plt

# Internal library
sys.path.append(os.path.join("..", ".."))
from models.bicycle import Bicycle  # noqa

if __name__ == "__main__":
    model = Bicycle(wheel_base=0.53)

    trajectory = SimpleGenerator(model)