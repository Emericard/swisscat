from typing import Union
from .point import Point


class Station:
    def __init__(self, name: str, x: int, y: int):
        self.name = None
        self.posGrid = Point(x, y)
        self.set_name(name)

    def set_name(self, name: str):
        if not isinstance(name, str):
            raise ValueError("Name should be a string.")
        self.name = name
