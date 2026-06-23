import math
import random
from typing import Optional, Tuple

import numpy as np

from type import Coordinate, Vector, create_coordinate, create_vector

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
        if location[0] == self.goal.location[0]:
            # TODO dejan: evaluate if this is needed or not.
            self.nearest_node.children.append(self.goal)
            self.goal.parent = self.nearest_node
        else:
            temp_node = Node(location=location)
            temp_node.parent = self.nearest_node
            self.nearest_node.children.append(temp_node)


    # sample a random point within grid lines
    def sample_a_point(self) -> Coordinate:
        x = random.randint(1, self.grid.shape[1])
        y = random.randint(1, self.grid.shape[0])
        return create_coordinate(x, y)


    # steer a distance stepsize from start to end location
    def steer_to_point(self, start_location: Coordinate, end_location: Coordinate) -> Coordinate:
        offset = self.rho * self.unit_vector(start_location, end_location)
        point = create_coordinate(start_location[0] + offset[0], start_location[1] + offset[1])
        point[0] = min(point[0], self.grid.shape[1] - 1)
        point[1] = min(point[1], self.grid.shape[0] - 1)
        return point


    # check if obstacle lies between start node and end point of the edge
    def does_obstacle_lie_between(self, start_location: Coordinate, end_location: Coordinate) -> bool:
        u_hat = self.unit_vector(start_location, end_location)
        test_point = create_coordinate(0.0, 0.0)
        for i in range(self.rho):
            test_point[0] = round(start_location[0] + i * u_hat[0])
            test_point[1] = round(start_location[1] + i * u_hat[1])
            
            # check if test_point lies within obstacle
            if self.grid[test_point[1].astype(np.int64), test_point[0].astype(np.int64)] == 1:
                return True
        return False


    # calculate unit vector for 2 points
    def unit_vector(self, start_location: Coordinate, end_location: Coordinate) -> Vector:
        v = create_vector(start_location[0] - end_location[0], start_location[1] - end_location[1])
        norm = math.sqrt(v[0]**2 + v[1]**2)
        return v / norm


    # find the nearest node from a given unconnected point (Euclidean distance)
    def find_nearest(self, root: Node, point: Node) -> None:
        if not root:
            return
        dist = self.distance(root, point)
        if dist <= self.nearest_distance:
            self.nearest_node = point
            self.nearest_distance = dist
        
        for child in root.children:
            self.find_nearest(child, point)


    # find Euclidean distance between a node and an XY point
    def distance(self, node1, point):
        difference = create_vector(node1[0] - point[0], node1[1] - point[1])
        norm = math.sqrt(difference[0]**2 + difference[1]**2)
        return norm


    # check if the goal has been found
    def is_goal_found(self, point) -> bool:
        if self.distance(self.goal, point) <= self.rho:
            return True
        return False


    # reset nearest node and nearest distance
    def reset_nearest_values(self):
        self.nearest_node = None
        self.nearest_distance = INFINITY


    # trace the path from the goal to start
    def retrace_path(self, goal: Node) -> None:
        if goal[0] == self.random_tree[0]:
            return
        self.number_of_waypoitns += 1
        current_point = create_coordinate(goal[0], goal[1])
        self.waypoints.insert(0, current_point)
        self.path_distance += self.rho
        self.retrace_path(goal.parent)
