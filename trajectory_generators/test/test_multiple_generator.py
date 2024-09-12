
# Standard library
import os
import sys
from matplotlib import pyplot as plt

# Internal library
sys.path.append(os.path.join("..", ".."))
from models.differential_drive import DifferentialDrive  # noqa
from trajectory_generators.simple_generator import SimpleGenerator  # noqa


if __name__ == "__main__":
    model = DifferentialDrive(wheel_base=0.53)

    trajectory = SimpleGenerator(model)

    trajectory.generate("global_trajectory.csv", nx=3, nu=2,
                        is_derivative=True)

    _, ax = plt.subplots()

    ax.plot(trajectory.x[:, 0], trajectory.x[:, 1], '--',
            label='Corner angle 90 degrees')

    trajectory.generate("global_trajectory_4.5.csv", nx=3, nu=2,
                        is_derivative=True)

    ax.plot(trajectory.x[:, 0], trajectory.x[:, 1], 'r',
            label='Corner angle 80 degrees')

    trajectory.generate("global_trajectory_4.0.csv", nx=3, nu=2,
                        is_derivative=True)

    ax.plot(trajectory.x[:, 0], trajectory.x[:, 1], 'b',
            label='Corner angle 70 degrees')

    trajectory.generate("global_trajectory_3.5.csv", nx=3, nu=2,
                        is_derivative=True)

    ax.plot(trajectory.x[:, 0], trajectory.x[:, 1], 'g',
            label='Corner angle 60 degrees')

    trajectory.generate("global_trajectory_3.0.csv", nx=3, nu=2,
                        is_derivative=True)

    ax.plot(trajectory.x[:, 0], trajectory.x[:, 1], 'g',
            label='Corner angle 50 degrees')

    ax.legend()

    ax.set_box_aspect(1)

    plt.show()
