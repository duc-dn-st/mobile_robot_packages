
# Standard library
import sys

sys.path.append("../../")

# Internal library
from models.differential_drive import DifferentialDrive 
from simulators.time_stepping import TimeStepping
from controllers.feedforward import FeedForward


if __name__ == "__main__":
    wheel_base = 0.53

    model = DifferentialDrive(wheel_base)

    controller = FeedForward(model, None)

    simulator = TimeStepping(model, None, controller, None)

    data = simulator.run(0.0)

    print(data[:, 0])

    print(data.shape)