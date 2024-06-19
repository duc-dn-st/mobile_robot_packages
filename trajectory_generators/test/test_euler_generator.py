
# Standard library
import os
import sys
import matplotlib.pyplot as plt

sys.path.append(os.path.join("..",".."))

# Standard library
import time

# Internal library
from models.differential_drive import DifferentialDrive
from trajectory_generators.euler_generator import EulerGenerator

if __name__ == "__main__":
    differential_drive = DifferentialDrive(wheel_base=0.255)

    trajectory_generator = EulerGenerator(differential_drive)

    initial_paths = [(1.0, 1.0), (3.0, 7.0), (8.0, 8.0), (9.0, 5.0), (10.0, 10.0)]

    start_time = time.time()

    position = [initial_paths[0]]

    position.extend(trajectory_generator.generate(initial_paths))

    position.append(initial_paths[-1])

    print("Execution time: ", time.time() - start_time)

    figure, ax = plt.subplots()

    ax.set_box_aspect(1)

    ax.plot([path[0] for path in initial_paths], [path[1] for path in initial_paths])

    ax.plot([path[0] for path in position], [path[1] for path in position])

    plt.show()

