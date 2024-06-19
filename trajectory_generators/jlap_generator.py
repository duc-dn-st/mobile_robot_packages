# Standard library
import math
import numpy as np

ACCELERATION_LIMIT = 0.177

JERK_LIMIT = ACCELERATION_LIMIT / (40 * 0.05)


class JlapGenerator: 
    def __init__(self):
        pass

    def generate(self, initial_paths, current_velocity=0.0):
        """! Generate a trajectory using the Jlap algorithm
        @param initial_paths: A list of paths to be followed
        @param current_velocity: The current velocity of the robot
        @return: A list of positions
        """
        position = []
        
        if len(initial_paths) <= 2: 
            self._generate_single(initial_paths, current_velocity)

        return position

    def _generate_single(self, initial_paths, current_velocity):
        """! Generate a single trajectory
        @param initial_paths: A list of paths to be followed
        @param current_velocity: The current velocity of the robot
        @return: A list of positions
        """
        start_acceleration = 0.0

        end_acceleration = 0.0 

        start_velocity = current_velocity 

        end_velocity = 0.0

        start = initial_paths[0]

        end = initial_paths[1]

        motion = self._generate_jlap_motion(start, end, start_velocity, end_velocity, start_acceleration, end_acceleration)

    def _generate_jlap_motion(self, start, end, start_acceleration, end_acceleration, start_velocity, end_velocity):
        """! Generate a motion profile using the Jlap algorithm
        @param start: The starting position
        @param end: The ending position
        @param start_acceleration: The starting acceleration
        @param end_acceleration: The ending acceleration
        @param start_velocity: The starting velocity
        @param end_velocity: The ending velocity
        @return: A list of positions
        """
        distance_vector = np.asarray(start) - np.asarray(end)

        line_length = np.linalg.norm(distance_vector)

        max_velocity = 1.1 * max(start_velocity, end_velocity)

        acceleration_displacement = self._calculate_phase_displament(start_acceleration, 0.0, start_velocity, max_velocity)

        deceleration_displacement = self._calculate_phase_displament(0.0, end_acceleration, max_velocity, end_velocity)

        displacement = acceleration_displacement[3] + deceleration_displacement[3]
        print("Displacement: ", displacement)
        if displacement > line_length: 
            pass

    @staticmethod
    def _calculate_phase_displament(start_acceleration, end_acceleration, start_velocity, end_velocity):
        """! Calculate the displacement of a phase
        @param start_acceleration: The starting acceleration
        @param end_acceleration: The ending acceleration
        @param start_velocity: The starting velocity
        @param end_velocity: The ending velocity
        @return: The displacement of the phase
        """
        delta = end_velocity - start_velocity

        sign = 0

        if delta > 0: 
            sign = 1

        elif delta < 0: 
            sign = -1 

        max_acceleration = ACCELERATION_LIMIT * sign

        jerk = JERK_LIMIT * sign

        t1 = 0.0

        t2 = 0.0

        t3 = 0.0

        if abs(delta) > 0:
            t1 = (max_acceleration - start_acceleration) / jerk 

            t3 = (end_acceleration - max_acceleration) / (-jerk)

            t2 = (1 / max_acceleration) * (delta - (1.0 / 2) * t1 * (max_acceleration + start_acceleration) - (1.0 / 2) * t3 * (max_acceleration + end_acceleration))

            if t2 < 0:
                max_acceleration = math.sqrt((1.0 / 2) * (start_acceleration * start_acceleration) + (end_acceleration * end_acceleration) + 2 * jerk * delta)

                max_acceleration = max_acceleration * sign

                t1 = (max_acceleration - start_acceleration) / jerk

                t3 = (end_acceleration - max_acceleration) / (-jerk)

                t2 = 0.0

        t1_acceleration = jerk * t1 

        t1_velocity = start_velocity + (1.0 / 2) * jerk * t1 * t1

        t1_displacement = start_velocity * t1 + (1.0 / 6) * jerk * t1 * t1 * t1

        t2_velocity = t1_velocity + t1_acceleration * t2

        t2_displacement = t1_displacement + t1_velocity * t2 + (1.0 / 2) * t1_acceleration * t2 * t2

        # t3_velocity = t2_velocity + t1_acceleration * t3 - (1.0 / 2) * jerk * t3 * t3 * t3 

        t3_displacement = t2_displacement + t2_velocity * t1 + (1.0 / 2) * t1_acceleration * t1 *t1 - (1.0 / 6) * jerk * t1 * t1 * t1

        return [t1, t2, t3, abs(t3_displacement)]
    