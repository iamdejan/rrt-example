import typing

from PIL import Image, ImageOps
import numpy as np
import matplotlib.pyplot as plt

from src.type import Coordinate, create_coordinate

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
    goal_region = plt.Circle((goal[0], goal[1]), step_size, color="b", fill=False)

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
    plt.show()


if __name__ == "__main__":
    main()
