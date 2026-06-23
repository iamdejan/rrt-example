import numpy as np

type Coordinate = np.ndarray
type Vector = np.ndarray

def create_coordinate(x: float, y: float) -> Coordinate:
    return np.array(object=[x, y], dtype=np.double)


def create_vector(x: float, y: float) -> Vector:
    return np.array(object=[x, y], dtype=np.double)
