# Standard library
import os
import numpy as np

# Internal library


class SimpleGenerator:
    """! Simple trajectory generator."""

    def __init__(self, model):
        """! Constructor."""
        self.x = None

        self.u = None

        self.t = None

        self._model = model

        current_directory = os.path.dirname(os.path.abspath(__file__))

        self._data_folder = os.path.join(current_directory, "data")

    def generate(self, file_name, nx, nu, is_derivative=False):
        """! Generate a simple trajectory.
        @param file_name<string>: The file name to save the
        generated trajectory
        @param nx<int>: The number of states
        @param nu<int>: The number of inputs
        @param is_derivative<bool>: The flag to indicate if the
        generated trajectory is a derivative
        @return None
        """
        data = np.genfromtxt(os.path.join(
            self._data_folder, file_name), delimiter=",")

        initial_index = 0

        if np.isnan(np.nan):
            initial_index = 1

        self.x = np.array(data[initial_index:, 1: 1 + nx])

        if len(data) > 1 + nx:
            self.u = self._retrieve_u(
                initial_index, data, nx, nu, is_derivative)

        self.t = np.array(data[initial_index:, 0])

        self.sampling_time = self.t[1] - self.t[0]

    # ==================================================================
    # PRIVATE METHODS
    # ==================================================================
    def _retrieve_u(self, initial_index, data, nx, nu, is_derivative):
        """! Retrieve the input at time t.
        @param t<float>: The time
        @return u<list>: The input
        """
        if not is_derivative:

            u = np.array(data[initial_index:, 1 + nx + 2 : 1 + nx + nu + 2])

        else:
            u = np.zeros((self._model.nu, len(data) - initial_index))

            u[0, :] = np.hypot(
                np.array(data[initial_index:, 1 + nx: 1 + nx + 1]),
                np.array(data[initial_index:, 1 + nx + 1: 1 + nx + 2]),
            ).reshape(-1)

            u[1, :] = np.array(
                data[initial_index:, 1 + nx + 2: 1 + nx + 3]).reshape(-1)

        return u
