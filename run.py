
# Standard library
import sys

sys.path.append("../../")

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

    trajectory.generate(None, None)

    controller = PurePursuit(trajectory)

    simulator = TimeStepping(model, trajectory, controller, None)

    simulator.run(0.0)

    print(simulator.x_out) 

    print(simulator.y_out)