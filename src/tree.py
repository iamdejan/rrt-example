from typing import Optional

from src.customtype import Coordinate


# Node class for the n-ary tree.
class Node:
    def __init__(self, location: Coordinate) -> None:
        self.children: list[Node] = []
        self.parent: Optional[Node] = None
        self.location: Coordinate = location
