import math
import random
from typing import Optional, cast

import numpy as np

from src.customtype import Coordinate, Vector, create_coordinate, create_vector
from src.tree import Node


INFINITY = 0x3f3f3f3f


class RRTAlgorithm:
    """
    Rapidly-exploring Random Tree (RRT) path planning algorithm.

    This class implements the classic RRT algorithm for finding obstacle-free
    paths in a 2D configuration space represented by a binary grid.

    Parameters
    ----------
    grid : np.ndarray
        A 2D binary numpy array representing the configuration space obstacles.
        1 indicates an obstacle, while 0 indicates free space.
    step_size : int
        The incremental distance (rho) to steer towards a sampled point.
    start : Coordinate
        The starting 2D coordinate [x, y] of the path.
    goal : Coordinate
        The destination 2D coordinate [x, y] of the path.
    number_of_iterations : int, optional
        The maximum number of search iterations (default is 100).
    """

    def __init__(
            self,
            grid: np.ndarray,
            step_size: int,
            start: Coordinate,
            goal: Coordinate,
            number_of_iterations: int = 1000,
        ) -> None:
        # Initialize the random search tree with the start node as the root.
        self.random_tree = Node(start)
        # Create a dedicated Node representing the goal.
        self.goal = Node(goal)
        # Keep track of the nearest node discovered during each sampling step.
        self.nearest_node: Optional[Node] = None
        # Store the 2D grid/map environment.
        self.grid = grid
        # Set the incremental step size (rho) for steering.
        self.rho = step_size
        # Total distance along the final planned path.
        self.path_distance = 0
        # Initialize nearest distance to infinity for minimum distance searches.
        self.nearest_distance: float = INFINITY
        # Total number of waypoints along the planned path.
        self.number_of_waypoints = 0
        # List of coordinates representing the sequential path from start to goal.
        self.waypoints: list[Coordinate] = []
        # Maximum allowed iterations to run the algorithm.
        self.number_of_iterations = number_of_iterations
        # Keep track of the last node successfully added to the tree.
        # This is used to correctly link the goal node as a child of the final path node.
        self.last_added_node: Optional[Node] = None

    def add_child(self, location: Coordinate) -> None:
        """
        Add a new node to the search tree.

        If the location matches the goal coordinate, the pre-existing self.goal
        node is linked to the last added node in the tree to preserve its reference.
        Otherwise, a new Node is created, linked to self.nearest_node, and tracked.

        Parameters
        ----------
        location : Coordinate
            The 2D coordinate where the child node should be placed.
        """
        # Check if the coordinates match the target goal coordinates.
        # This allows us to reuse the self.goal node instance and set its parent.
        if np.allclose(location, self.goal.location):
            # The parent of the goal is set to the last successfully added tree node.
            self.goal.parent = self.last_added_node
            # Append the goal node to the children list of the last added node.
            if self.last_added_node is not None:
                self.last_added_node.children.append(self.goal)
        else:
            # Create a new tree node at the specified coordinate.
            temp_node = Node(location=location)
            # Set its parent to the nearest node found in the tree.
            temp_node.parent = self.nearest_node
            # Append the new node to the nearest node's children list.
            cast(Node, self.nearest_node).children.append(temp_node)
            # Keep track of this newly added node as the most recent addition.
            self.last_added_node = temp_node

    def sample_a_point(self) -> Coordinate:
        """
        Sample a random integer coordinate within the limits of the grid.

        Returns
        -------
        Coordinate
            A randomly sampled 2D coordinate [x, y].
        """
        # Sample x and y within grid boundaries.
        # This provides a random coordinate in the search space.
        x = random.randint(0, self.grid.shape[1] - 1)
        y = random.randint(0, self.grid.shape[0] - 1)
        return create_coordinate(x, y)

    def steer_to_point(self, start_node: Node, end_location: Coordinate) -> Coordinate:
        """
        Steer from start_node location towards end_location by step_size.

        Parameters
        ----------
        start_node : Node
            The node from which we are steering.
        end_location : Coordinate
            The target coordinate we are steering towards.

        Returns
        -------
        Coordinate
            The new coordinate reached after steering.
        """
        # Calculate the unit vector pointing from the start node to the target.
        u_hat = self.unit_vector(start_node, end_location)
        # Compute the offset by scaling the unit vector by the step size (rho).
        offset = self.rho * u_hat
        # Generate the new coordinate.
        point = create_coordinate(start_node.location[0] + offset[0], start_node.location[1] + offset[1])
        # Ensure the steered point coordinates do not exceed the upper grid bounds.
        point[0] = min(point[0], self.grid.shape[1] - 1)
        point[1] = min(point[1], self.grid.shape[0] - 1)
        # Ensure the steered point coordinates are not negative.
        point[0] = max(point[0], 0.0)
        point[1] = max(point[1], 0.0)
        return point

    def does_obstacle_lie_between(self, start_node: Node, end_location: Coordinate) -> bool:
        """
        Check if an obstacle lies on the straight-line segment.

        The segment is defined between the starting node and the target location,
        sampled at unit intervals up to the step size.

        Parameters
        ----------
        start_node : Node
            The start node of the segment.
        end_location : Coordinate
            The end coordinates of the segment.

        Returns
        -------
        bool
            True if any sampled point along the segment collides with an obstacle,
            or goes out of grid bounds; False otherwise.
        """
        # Calculate the unit vector pointing in the direction of growth.
        u_hat = self.unit_vector(start_node, end_location)
        # Iterate along the path from 0 to rho to check for any obstacle collisions.
        for i in range(self.rho + 1):
            # Calculate and round the coordinates of the intermediate point.
            x_idx = int(round(start_node.location[0] + i * u_hat[0]))
            y_idx = int(round(start_node.location[1] + i * u_hat[1]))
            # Check if the calculated point is out of the grid boundaries.
            if x_idx < 0 or x_idx >= self.grid.shape[1] or y_idx < 0 or y_idx >= self.grid.shape[0]:
                return True
            # Check if the grid cell at the index is an obstacle (represented by 1).
            if self.grid[y_idx, x_idx] == 1:
                return True
        return False

    def unit_vector(self, start_node: Node, end_location: Coordinate) -> Vector:
        """
        Calculate the unit vector pointing from start_node to end_location.

        Parameters
        ----------
        start_node : Node
            The start node.
        end_location : Coordinate
            The target coordinate.

        Returns
        -------
        Vector
            The unit vector pointing from start_node towards end_location.
        """
        # Construct the difference vector pointing from start to end (end - start).
        # This ensures RRT grows forward towards the sampled goal/point.
        v = create_vector(end_location[0] - start_node.location[0], end_location[1] - start_node.location[1])
        # Compute the Euclidean norm of the vector.
        norm = math.sqrt(v[0]**2 + v[1]**2)
        # Avoid division by zero if the points are identical.
        if norm == 0.0:
            return create_vector(0.0, 0.0)
        return v / norm

    def find_nearest(self, root: Node, point: Coordinate) -> None:
        """
        Recursively traverse the tree to find the nearest Node to the point.

        The nearest node is stored in self.nearest_node, and its distance
        is stored in self.nearest_distance.

        Parameters
        ----------
        root : Node
            The current root node to search from.
        point : Coordinate
            The coordinates of the target point.
        """
        if not root:
            return

        # Calculate Euclidean distance from the current node to the target point.
        dist = self.distance(root, point)
        # If this distance is smaller than the minimum found so far, update the records.
        if dist < self.nearest_distance:
            self.nearest_node = root
            self.nearest_distance = dist

        # Recursively search all child nodes.
        for child in root.children:
            self.find_nearest(child, point)

    def distance(self, node1: Node, point: Coordinate) -> float:
        """
        Calculate the Euclidean distance between a node and a coordinate.

        Parameters
        ----------
        node1 : Node
            The node.
        point : Coordinate
            The 2D coordinate.

        Returns
        -------
        float
            The Euclidean distance.
        """
        # Compute the vector difference between the node's location and the target coordinate.
        difference = create_vector(node1.location[0] - point[0], node1.location[1] - point[1])
        # Return the magnitude/length of the difference vector.
        return float(math.sqrt(difference[0]**2 + difference[1]**2))

    def is_goal_found(self, point: Coordinate) -> bool:
        """
        Check if the given coordinate is within step_size of the goal.

        Parameters
        ----------
        point : Coordinate
            The coordinate to check.

        Returns
        -------
        bool
            True if the distance to the goal is less than or equal to step_size.
        """
        # The goal is reached if we can steer directly to it in a single step.
        return self.distance(self.goal, point) <= self.rho

    def reset_nearest_values(self) -> None:
        """
        Reset the nearest node tracking variables for a new search iteration.
        """
        # Clear the nearest node reference.
        self.nearest_node = None
        # Reset the minimum distance back to infinity.
        self.nearest_distance = INFINITY

    def retrace_path(self, goal: Optional[Node]) -> None:
        """
        Recursively trace parent pointers from the goal back to the start root.

        This constructs the ordered list of coordinates (waypoints) and
        computes the path distance.

        Parameters
        ----------
        goal : Node, optional
            The node to retrace from.
        """
        if goal is None:
            return
        # Stop retracing once we reach the starting root node.
        if goal is self.random_tree:
            return
        # Increment the count of waypoints.
        self.number_of_waypoints += 1
        # Extract and save the coordinate of the current path node.
        current_point = create_coordinate(goal.location[0], goal.location[1])
        self.waypoints.insert(0, current_point)
        # Add the step length to the total distance.
        self.path_distance += self.rho
        # Recurse on the parent node.
        self.retrace_path(goal.parent)
