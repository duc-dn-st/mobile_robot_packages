
# Standard library
import sys


# Internal library
sys.path.append("../../")
from models.differential_drive import DifferentialDrive # noqa
from simulators.time_stepping import TimeStepping # noqa
from controllers.feedforward import FeedForward # noqa


if __name__ == "__main__":
    wheel_base = 0.53

    model = DifferentialDrive(wheel_base)

    controller = FeedForward(model, None)

    simulator = TimeStepping(model, None, controller, None)

    data = simulator.run(0.0)

    print(data[:, 0])

    print(data.shape)