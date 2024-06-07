# Standard library 
import numpy as np


class TimeStepping: 
    t_start = 0

    t_max = 60 

    dt = 0.05

    def __init__(self, model, trajectory, controller, observer):
        self.t_out = []
        
        self.x_out = []

        self.y_out = []

        self.u_out = []

        self.model = model

        self.trajectory = trajectory
        
        self.controller = controller 

        self.observer = observer 

    def run(self, q0):
        self.t_out = np.linspace(TimeStepping.t_start, TimeStepping.t_max, int((1 / TimeStepping.dt) * TimeStepping.t_max))

        nt = self.t_out.shape[-1]

        self.x_out = np.zeros([self.model.nx, nt])

        self.y_out = np.zeros([self.model.nx, nt])
        
        # Intialize time stepping
        self.x_out[:, 0] = q0

        self.y_out[:, 0] = q0

        self.u_out = np.zeros([self.model.nu, nt])

        self.controller.initialize()

        for index in range(nt):
            y_m = self.y_out[:, index]

            u_m = self.u_out[:, index]

            status, self.u_out[:, index] = self.controller.execute(y_m, u_m, index)

            if not status: 
                self.u_out[:, index] = np.zeros([self.model.nu, 1])
            
            x_m = self.model.function(self.x_out[:, index], self.u_out[:, index], TimeStepping.dt)

            y_m = x_m

            if index < nt - 1:
                self.x_out[:, index + 1] = x_m 

                self.y_out[:, index + 1] = y_m