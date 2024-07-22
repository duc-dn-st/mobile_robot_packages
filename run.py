
# Standard library
import sys
from matplotlib import pyplot as plt

# Internal library
from environments.graph import Graph
from controllers.purepursuit import PurePursuit
from simulators.time_stepping import TimeStepping
from models.differential_drive import DifferentialDrive 
from trajectory_generators.simple_generator import SimpleGenerator


if __name__ == "__main__":
    wheel_base = 0.53

    environment = Graph()

    model = DifferentialDrive(wheel_base)

    trajectory = SimpleGenerator(environment)

    trajectory.generate("eight_curve.csv", nx=3, nu=2)

    controller = PurePursuit(model, trajectory)

    simulator = TimeStepping(model, trajectory, controller, None)

    simulator.run(0.0)

    figure, ax = plt.subplots()

    ax.set_box_aspect(1)

    ax.plot(simulator.x_out[0, :], simulator.x_out[1, :])

    ax.plot([path[0] for path in trajectory.x], [path[1] for path in trajectory.x])

    plt.show()