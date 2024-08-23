# Standard library
import os
import sys
import time
import matplotlib.pyplot as plt

# Internal library
sys.path.append(os.path.join("..", ".."))
from models.differential_drive import DifferentialDrive # noqa
from trajectory_generators.euler_generator import EulerGenerator # noqa


if __name__ == "__main__":
    differential_drive = DifferentialDrive(wheel_base=0.51)

    trajectory_generator = EulerGenerator(differential_drive)

    initial_paths = [(0.0, 0.0), (5.0, 0.0), (5.0, 5.0)]

    start_time = time.time()

    position = [initial_paths[0]]

    position.extend(trajectory_generator.generate(initial_paths))

    position.append(initial_paths[-1])

    print("Execution time: ", time.time() - start_time)

    figure, ax = plt.subplots()

    ax.set_box_aspect(1)

    ax.plot([path[0] for path in initial_paths],
            [path[1] for path in initial_paths])

    ax.plot([path[0] for path in position], [path[1] for path in position])

    plt.show()
