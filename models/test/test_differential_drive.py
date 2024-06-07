
# Standard library
import sys

sys.path.append("../../")

# Internal library 
from models.differential_drive import DifferentialDrive


if __name__ == "__main__":
    wheel_base = 0.53 / 2

    model = DifferentialDrive(wheel_base)

    state = [0.0, 0.0, 0.0]

    input = [1.0, 0.0]

    dt = 0.05 # ms

    data = model.function(state, input, dt)

    print(data)