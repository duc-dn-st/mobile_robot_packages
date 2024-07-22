# Standard library
import os 
import math
import numpy as np 

# Internal library 


class SimpleGenerator: 
    """! Simple trajectory generator."""

    def __init__(self, environment):
        """! Constructor.
        """
        self.x = None 

        self.u = None
        
        self.t = None

        current_directory = os.path.dirname(os.path.abspath(__file__))

        self._data_folder = os.path.join(current_directory, 'data')

    def generate(self, file_name, nx, nu):
        """! Generate a simple trajectory.
        @param file_name<string>: The file name to save the generated trajectory
        """
        data = np.genfromtxt(os.path.join(self._data_folder, file_name), delimiter=',')
        
        self.x = np.array(data[0:, 1:1+nx])

        if len(data) > 1 + nx:
            self.u = np.array(data[0:, 1+nx:1+nx+nu])
        
        self.t = np.array(data[0:, 0])




        

