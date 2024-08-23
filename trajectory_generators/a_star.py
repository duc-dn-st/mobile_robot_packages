# External library
from extremitypathfinder import PolygonEnvironment


class AStar:
    def __init__(self, environment):
        self.environment = environment

        self.generator = PolygonEnvironment()

        self.generator.store(self.environment.boundaries,
                             self.environment.obstacles, validate=False)

    def generate(self, start, end):
        path, length = self.generator.find_shortest_path(start, end)

        return path, length
