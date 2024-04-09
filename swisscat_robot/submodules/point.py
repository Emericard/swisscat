import math
from typing import Union


class Point:
    def __init__(self, x: Union[float, int] = 0, y: Union[float, int] = 0):
        if not isinstance(x, (int, float)) or not isinstance(y, (int, float)):
            raise ValueError("Both x and y should be numbers (int or float).")

        self.x = x
        self.y = y

    def __add__(self, other: 'Point') -> 'Point':
        if isinstance(other, Point):
            return Point(self.x + other.x, self.y + other.y)
        else:
            raise ValueError("Can't add Point with non-Point type")

    def __sub__(self, other: 'Point') -> 'Point':
        if isinstance(other, Point):
            return Point(self.x - other.x, self.y - other.y)
        else:
            raise ValueError("Can't subtract non-Point type from Point")

    def __mul__(self, scalar: Union[int, float]) -> 'Point':
        if isinstance(scalar, (int, float)):
            return Point(self.x * scalar, self.y * scalar)
        else:
            raise ValueError("Can only multiply Point by a scalar (int or float)")

    def __truediv__(self, scalar: Union[int, float]) -> 'Point':
        if isinstance(scalar, (int, float)):
            if scalar == 0:
                raise ValueError("Cannot divide by zero")
            return Point(self.x / scalar, self.y / scalar)
        else:
            raise ValueError("Can only divide Point by a scalar (int or float)")

    def __eq__(self, other: 'Point') -> bool:
        if isinstance(other, Point):
            return self.x == other.x and self.y == other.y
        return False

    def __repr__(self) -> str:
        return f"Point({self.x}, {self.y})"

    def __rmul__(self, scalar: Union[int, float]) -> 'Point':
        return self.__mul__(scalar)

    def __neg__(self) -> 'Point':
        return Point(-self.x, -self.y)

    def distance_to(self, other: 'Point') -> float:
        if not isinstance(other, Point):
            raise ValueError("Can only calculate distance to another Point")
        return math.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)

    def distance_from_origin(self) -> float:
        return math.sqrt(self.x ** 2 + self.y ** 2)

    def move(self, dx: Union[float, int], dy: Union[float, int]) -> None:
        if not isinstance(dx, (int, float)) or not isinstance(dy, (int, float)):
            raise ValueError("Both dx and dy should be numbers (int or float).")

        self.x += float(dx)
        self.y += float(dy)

    def set_coordinates(self, new_x: Union[float, int], new_y: Union[float, int]):
        if not isinstance(new_x, (int, float)) or not isinstance(new_y, (int, float)):
            raise ValueError("Both new_x and new_y should be numbers (int or float).")

        self.x = float(new_x)
        self.y = float(new_y)

    def get_coordinates(self):
        return self.x, self.y
