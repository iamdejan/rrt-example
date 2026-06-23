import typing

from PIL import Image, ImageOps
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

from src.customtype import Coordinate, create_coordinate
from src.rrt import RRTAlgorithm
from src.tree import Node

def convert_to_numpy_image(file_name: str) -> np.ndarray:
    image = typing.cast(Image.Image, Image.open(file_name))
    image = ImageOps.grayscale(image)

    # convert to NumPy image
    np_image = np.array(image)
    np_image = ~np_image
    np_image[np_image > 0] = 1

    # save to NPY file
    np.save("cspace.npy", np_image)
    return np_image


def main() -> None:
    grid = convert_to_numpy_image("cspace.png")

    # load the grid, set start and goal <x, y> positions, number of iterations, step size
    start: Coordinate = create_coordinate(100.0, 100.0)
    goal: Coordinate = create_coordinate(830.0, 610.0)
    step_size = 40
    goal_region = Circle((goal[0], goal[1]), step_size, color="b", fill=False)

    fig = plt.figure()
    plt.title("RRT Algorithm")
    plt.imshow(grid, cmap="binary")
    plt.plot(start[0], start[1], "ro")
    plt.plot(goal[0], goal[1], "bo")

    ax = fig.gca()
    ax.add_patch(goal_region)
    plt.xlabel("X-axis $(m)$")
    plt.ylabel("Y-axis $(m)$")

    plt.tight_layout()

    # RRT
    rrt = RRTAlgorithm(grid, step_size, start, goal)
    for i in range(rrt.number_of_iterations):
        rrt.reset_nearest_values()
        sampled_point = rrt.sample_a_point()
        rrt.find_nearest(rrt.random_tree, sampled_point)
        rrt.nearest_node = typing.cast(Node, rrt.nearest_node) # temporary measure
        new_point = rrt.steer_to_point(rrt.nearest_node, sampled_point)
        does_obstacle_lie_between_points = rrt.does_obstacle_lie_between(rrt.nearest_node, new_point)
        print(f"iteration {i} -> sampled_point = {sampled_point}, rrt.nearest_node.location = {rrt.nearest_node.location}, does_obstacle_lie_between_points = {does_obstacle_lie_between_points}")
        if not does_obstacle_lie_between_points:
            rrt.add_child(new_point)
            plt.pause(0.10)
            plt.plot([rrt.nearest_node.location[0], new_point[0]], [rrt.nearest_node.location[1], new_point[1]], "go", linestyle="--")
            if rrt.is_goal_found(new_point):
                rrt.add_child(goal)
                print("Goal found")
                break

    rrt.retrace_path(rrt.goal)
    rrt.waypoints.insert(0, start)
    print(f"Number of waypoints: {rrt.number_of_waypoints}")
    print(f"Path distance (m): {rrt.path_distance}")
    print(f"Waypoints: {rrt.waypoints}")


    for i in range(len(rrt.waypoints)-1):
        plt.plot([rrt.waypoints[i][0], rrt.waypoints[i + 1][0]], [rrt.waypoints[i][1], rrt.waypoints[i + 1][1]], "ro", linestyle="--")
        plt.pause(0.10)


if __name__ == "__main__":
    main()
