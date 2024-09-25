import numpy as np
from dataclasses import dataclass, field
from typing import List
from shapely import Polygon as ShapelyPolygon

# Helper class for a circle
@dataclass
class Circle:
    center: np.ndarray = field()
    radius: float = field()

    def __post_init__(self):
        # Ensure that center is a numpy array
        if not (isinstance(self.center, np.ndarray) or isinstance(self.center, list) or isinstance(self.center, tuple)):
            raise TypeError(f"Expected 'center' to be a list, tuple or numpy array, but got '{type(self.center).__name__}' instead.")
        elif isinstance(self.center, (list, tuple)):
            self.center = np.array(self.center)

        # Ensure that center is a vector with 2 elements
        if not self.center.shape == (2,):
            if self.center.shape == (1, 2):
                self.center = self.center.reshape(-1,)
            else:
                raise ValueError(f"Expected 'center' to be a vector with 2 elements, but got a numpy array of shape {self.center.shape} elements instead.")

        # Ensure that radius is a positive float
        if not isinstance(self.radius, (int, float)):
            raise TypeError(f"Expected 'radius' to be a float, but got '{type(self.radius).__name__}' instead.")
        elif self.radius < 0:
            raise ValueError("Radius must be non-negative.")

    def contains(self, point: np.ndarray) -> bool:
        """Check if a point is inside the circle"""
        return np.linalg.norm(self.center - point) <= self.radius

@dataclass
class Polygon:
    """
    Helper class for a polygon. The polygon is defined by a list of points, each connecting to the next point in the list (wrapping around of last-first).
    """
    points: List[np.ndarray] = field()

    def __post_init__(self):
        # Ensure that points is a list of numpy arrays
        if not (isinstance(self.points, list) or isinstance(self.points, tuple)):
            raise TypeError(f"Expected 'points' to be a list or tuple, but got '{type(self.points).__name__}' instead.")
        elif isinstance(self.points, tuple):
            self.points = list(self.points)

        # Ensure that the polygon has at least 3 points
        if len(self.points) < 3:
            raise ValueError("A polygon must have at least 3 points.")
        
        # Ensure that each element in points is a numpy array
        for i, point in enumerate(self.points):
            if not (isinstance(point, np.ndarray) or isinstance(point, list) or isinstance(point, tuple)):
                raise TypeError(f"Element at index {i} is not a list, tuple or numpy array, but got '{type(point).__name__}' instead.")
            elif isinstance(point, (list, tuple)):
                self.points[i] = np.array(point)
            
        # Ensure the polygon is not self-intersecting
        if not ShapelyPolygon([point.tolist() for point in self.points]).is_simple:
            raise ValueError("Polygon is self-intersecting. Ensure all points were provided in the correct order.")
            
if __name__ == "__main__":
    s = Polygon([(0, 0), (1, 0), (1, 1), (0, 1)])