from PIL import Image, ImageOps
import numpy as np
import matplotlib.pyplot as plt


def main() -> None:
    image = Image.open("cspace.png")
    image = ImageOps.grayscale(image)

    # convert to NumPy image
    np_image = np.array(image)
    np_image = ~np_image
    np_image[np_image > 0] = 1

    # save to NPY file
    np.save("cspace.npy", np_image)

    # open via Matplotlib
    np_image = np.load("cspace.npy")
    plt.imshow(np_image, cmap="binary")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
