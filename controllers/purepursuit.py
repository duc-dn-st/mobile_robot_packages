
# Standard library
import math
import numpy as np 


class PurePursuit:
    lookahead_distance = 2.0

    lookahead_gain = 0.1

    k = 1

    def __init__(self, trajectory):
        """! Constructor
        """
        self.trajectory = trajectory

        self.old_nearest_point_index = None

    def _initialize(self):
        self.v = 10 / 36 # m/s

    def _apply_proportional_control(k_p, target, current):
        """! Apply proportional control
        """
        a = k_p * (target - current)

        return a

        pass
    
    @staticmethod
    def _calculate_distance(x, y, target_x, target_y):
        distance_x = x - target_x 

        distance_y = y - target_y
        
        return math.hypot(distance_x, distance_y)

    def execute(self, state, input, previous_index):
        status = True

        index, lookahead_distance = self._search_target_index(state)

        if previous_index >= index:
            index = previous_index

        if index < len(self.trajectory.x):
            toward_x = self.trajectory.x[index]

            toward_y = self.trajectory.y[index]

        else: 
            toward_x = self.trajectory.x[-1]

            toward_y = self.trajectory.y[-1]

        output = input

        return status, output

    def _search_target_index(self, state):
        if self.old_nearest_point_index is None: 
            distance_x = [state[0] - x for x in self.trajectory.x]

            distance_y = [state[1] - y for y in self.trajectory.y]

            distance = np.hypot(distance_x, distance_y)

            index = np.argmin(distance)

        else: 
            index = self.old_nearest_point_index

            distance_at_index = self._calculate_distance(state[0], state[1], self.trajectory.x[index], self.trajectory.y[index])

            while True: 
                distance_next_index = self._calculate_distance(state[0], state[1], self.trajectory.x[index + 1], self.trajectory.y[index + 1])

                if distance_at_index < distance_next_index:
                    break

                index = index + 1 if index + 1 < len(self.trajectory.x) else index

                distance_at_index = distance_next_index

        self.old_nearest_point_index = index

        lookahead_distance = PurePursuit.k * self.v + PurePursuit.lookahead_distance

        while lookahead_distance > self._calculate_distance(state[0], state[1], self.trajectory.x[index], self.trajectory.y[index]):
            if index + 1 >= len(self.trajectory.x):
                break

            index += 1

        return index, lookahead_distance 