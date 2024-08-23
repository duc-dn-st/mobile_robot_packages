# Standard library


class Graph:
    def __init__(self):
        self.boundaries = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]

        self.obstacles = [
            [
                (3.0, 3.0),
                (3.0, 7.0),
                (7.0, 7.0),
                (7.0, 3.0),
            ],
        ]
