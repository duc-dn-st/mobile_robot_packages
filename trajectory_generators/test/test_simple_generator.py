
# Standard library
import os
import sys
import numpy as np
from matplotlib import pyplot as plt

# Internal library
sys.path.append(os.path.join("..", ".."))
from models.differential_drive import DifferentialDrive # noqa
from trajectory_generators.simple_generator import SimpleGenerator # noqa


if __name__ == "__main__":
    model = DifferentialDrive(wheel_base=0.53)

    trajectory = SimpleGenerator(model)

    trajectory.generate("global_trajectory.csv", nx=3, nu=2,
                        is_derivative=True)

    initial_paths = [(0.0, 0.0), (5.0, 0.0), (5.0, 5.0)]

    print(trajectory.x)

    print(trajectory.u)

    print(trajectory.t)

    dudt = np.zeros(trajectory.u.shape)

    print(trajectory.u.shape)

    print(dudt.shape)

    for index in range(len(trajectory.t)):
        dudt[:, index] = (trajectory.u[:, index] - trajectory.u[:, index - 1]
                          ) / (trajectory.t[index] - trajectory.t[index - 1])

    figure, ax = plt.subplots()

    ax.set_box_aspect(1)

    ax.plot(trajectory.x[:, 0], trajectory.x[:, 1])

    _, (ax1, ax2) = plt.subplots(1, 2)

    ax1.plot(trajectory.t, trajectory.u[0, :])

    ax2.plot(trajectory.t, trajectory.u[1, :])

    _, (ax3, ax4) = plt.subplots(1, 2)

    ax3.plot(trajectory.t, dudt[0, :])

    ax4.plot(trajectory.t, dudt[1, :])

    print("max of dudt[1, :]: ", np.max(dudt[1, :]))

    plt.show()
