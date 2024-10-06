
# Standard library
import os
import sys
from matplotlib import pyplot as plt

# Internal library
sys.path.append(os.path.join("..", ".."))
from models.bicycle import Bicycle # noqa
from trajectory_generators.simple_generator import SimpleGenerator # noqa


def _save_trajectories(file_name, trajectory):
    """! Save the generated trajectory.
    @param file_name<string>: The file name to save the
    generated trajectory
    @param trajectory<SimpleGenerator>: The generated trajectory
    @return None
    """
    with open(file_name, "w") as file:
        file.write("Time [s], x [m], y [m], theta [rad], v_rear [m/s], w_rear [rad/s], v_front [m/s], delta [rad]\n")

        for index in range(len(trajectory.t)):
            v_front, delta = trajectory._model.calculate_front_axle_velocity(
                trajectory.x[index, :], trajectory.u[:, index])

            file.write("{}, {}, {}, {}, {}, {}, {}, {}\n".format(
                trajectory.t[index],
                trajectory.x[index, 0],
                trajectory.x[index, 1],
                trajectory.x[index, 2],
                trajectory.u[0, index],
                trajectory.u[1, index],
                v_front,
                delta
            ))


if __name__ == "__main__":
    model = Bicycle(lengh_base=1.0)

    trajectory = SimpleGenerator(model)

    trajectory.generate("uturn_1s.csv", nx=3, nu=2,
                        is_derivative=True)

    figure, ax = plt.subplots()

    ax.set_box_aspect(1)

    ax.plot(trajectory.x[:, 0], trajectory.x[:, 1], "bo")

    _, (ax1, ax2) = plt.subplots(1, 2)

    ax1.plot(trajectory.t, trajectory.u[0, :])

    ax2.plot(trajectory.t, trajectory.u[1, :])

    ax1.set_xlabel("Time [s]")

    ax1.set_ylabel("v_r [m/s]")

    ax2.set_xlabel("Time [s]")

    ax2.set_ylabel("omega_r [rad/s]")

    plt.tight_layout()

    file_path = os.path.join(trajectory._data_folder, "bicycle_uturn_1s.csv")
    
    _save_trajectories(file_path, trajectory)

    plt.show()
