from typing import Optional, Tuple

import numpy as np

from type import Coordinate, Vectpr

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
            step_size: float,
            start: Coordinate,
            goal: Coordinate,
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


    # add the point to the nearest goal and add goal when reached
    def add_child(self, location: Coordinate) -> None:
        pass


    # sample a random point within grid lines
    def sample_a_point(self) -> Coordinate:
        pass


    # steer a distance stepsize from start to end location
    def steer_to_point(self, target: Coordinate) -> None:
        pass


    # check if obstacle lies between start node and end point of the edge
    def does_obstacle_lie_between(self, start_location: Coordinate, end_location: Coordinate) -> bool:
        pass


    # calculate unit vector for 2 points
    def unit_vector(self, start_location: Coordinate, end_location: Coordinate) -> Vectpr:
        pass


    # find the nearest node from a given unconnected point (Euclidean distance)
    def find_nearest(self, root: Node, point: Node) -> Node:
        pass


    # find Euclidean distance between a node and an XY point
    def distance(self, node1, point):
        pass


    # check if the goal has been found
    def is_goal_found(self, point):
        pass


    # reset nearest node and nearest distance
    def reset_nearest_values(self):
        pass


    # trace the path from the goal to start
    def retrace_path(self, goal):
        pass
