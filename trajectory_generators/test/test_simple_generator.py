
# Standard library
import os
import sys
import matplotlib.pyplot as plt

sys.path.append(os.path.join("..",".."))

# Internal library
from trajectory_generators.simple_generator import SimpleGenerator


if __name__ == "__main__":

    trajectory = SimpleGenerator()

    x, y, t, data = trajectory.generate("global_trajectory.csv")

    a_star_paths = [[1.0, 1.0], [3.0, 7.0], [8.0, 8.0]]

    euler_paths = [[1.0, 1.0], [2.738366484819262, 6.215099454457786], [3.101441758695337, 6.885088579532767], [3.8112910903771104, 7.162258218075421], [8.0, 8.0]]

    figure, ax = plt.subplots()

    ax.set_box_aspect(1)

    ax.plot(x, y)

    ax.plot([path[0] for path in euler_paths], [path[1] for path in euler_paths])

    ax.plot([path[0] for path in a_star_paths], [path[1] for path in a_star_paths])
    
    ax.set_xlabel('X')

    ax.set_ylabel('Y')

    ax.set_title('Trajectory Comparison')

    ax.legend(['Generated Trajectory', 'Euler Paths', 'A* Paths'])

    figure2, ax2 = plt.subplots()

    ax2.plot(t, data[1:, 7])

    ax2.set_xlabel('Time')

    ax2.set_ylabel('Path Velocity')

    ax2.set_title('Path Velocity over Time')

    plt.show()