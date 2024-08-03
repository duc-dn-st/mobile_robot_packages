# Standard library
from matplotlib import pyplot as plt

# Internal library
from environments.graph import Graph
from simulators.time_stepping import TimeStepping
from models.differential_drive import DifferentialDrive
from controllers.purepursuit.purepursuit import PurePursuit
from trajectory_generators.simple_generator import SimpleGenerator


if __name__ == "__main__":
    wheel_base = 0.53

    environment = Graph()

    model = DifferentialDrive(wheel_base)

    trajectory = SimpleGenerator(environment)

    trajectory.generate("global_trajectory.csv", nx=3, nu=2)

    controller = PurePursuit(model, trajectory)

    simulator = TimeStepping(model, trajectory, controller, None)

    simulator.run(0.0)

    figure, (ax1, ax2) = plt.subplots(1, 2)

    ax1.set_box_aspect(1)

    ax1.plot(simulator.x_out[0, :], simulator.x_out[1, :], "r")

    ax1.plot(
        [path[0] for path in trajectory.x],
        [path[1] for path in trajectory.x],
        "--b"
    )

    ax2.plot(simulator.t_out, simulator.u_out[0, :])

    plt.show()
