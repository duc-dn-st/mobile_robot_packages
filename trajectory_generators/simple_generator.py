# Standard library
import os 
import math
import numpy as np 

# Internal library 


class SimpleGenerator: 
    """! Simple trajectory generator."""

    def __init__(self):
        """! Constructor.
        """
        current_directory = os.path.dirname(os.path.abspath(__file__))

        self._data_folder = os.path.join(current_directory, 'data')

    def generate(self, file_name):
        """! Generate a simple trajectory.
        @param file_name<string>: The file name to save the generated trajectory
        """
        data = np.genfromtxt(os.path.join(self._data_folder, file_name), delimiter=',')
        
        x = data[1:, 1]
        
        y = data[1:, 2]
        
        t = data[1:, 0]

        return x, y, t, data



        

