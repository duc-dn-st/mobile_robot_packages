class FeedForward:
    def __init__(self, model, trajectory):
        self.model = model

        self.trajectory = trajectory

    def initialize(self):
        pass

    def execute(self, state, input, index):
        status = True

        u = self.trajectory.u[:, index]

        return status, u
