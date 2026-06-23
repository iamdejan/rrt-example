from typing import Optional, Tuple

import numpy as np


type Coordinate = Tuple[float, float]


INFINITY = 0x3f3f3f3f


# Node class for the n-ary tree.
class Node:
    def __init__(self, location: Tuple[float, float] = ()) -> None:
        self.children: list[Node] = []
        self.parent: Optional[Node] = None
        self.location: Coordinate = location


class RRTAlgorithm:
    def __init__(
            self,
            grid: np.ndarray,
            step_size: float, # TODO dejan: check if can be replaced with int or not
            start: Coordinate,
            goal: Coordinate,
            goal_tolerance: Coordinate=(0.0, 0.0),
            number_of_iterations: int=100,
        ):
        self.random_tree = Node(start)
        self.goal = Node(goal)
        self.nearest_node: Optional[Node] = None
        self.grid = grid
        self.rho = step_size
        self.path_distance = 0
        self.nearest_distance = INFINITY
        self.number_of_waypoitns = 0
        self.waypoints = []
        self.number_of_iterations = number_of_iterations
