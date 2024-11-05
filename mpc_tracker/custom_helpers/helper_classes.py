import numpy as np
from dataclasses import dataclass, field
from typing import List, Tuple
from shapely import Polygon as ShapelyPolygon

# Helper class for a circle
@dataclass
class Circle:
    # Public attributes
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
    # Public attributes
    points: List[np.ndarray] = field()

    # Private attributes
    _center: np.ndarray = field(init=False)
    _set_equations: List[str] = field(init=False, default_factory=list)  # Initialize each instance with its own list

    def __post_init__(self):
        # Ensure that points is a list (of numpy arrays)
        if not (isinstance(self.points, list) or isinstance(self.points, tuple) or isinstance(self.points, np.ndarray)):
            raise TypeError(f"Expected 'points' to be a list, tuple or np.ndarray, but got '{type(self.points).__name__}' instead.")
        elif isinstance(self.points, np.ndarray):
            self.points = list(self.points)
        elif isinstance(self.points, tuple):
            self.points = list(self.points)
        
        # Ensure that each element in points is a numpy array
        for i, point in enumerate(self.points):
            if not (isinstance(point, np.ndarray) or isinstance(point, list) or isinstance(point, tuple)):
                raise TypeError(f"Element at index {i} is not a list, tuple or numpy array, but got '{type(point).__name__}' instead.")
            elif isinstance(point, (list, tuple)):
                self.points[i] = np.array(point)

        # Eliminate collinear points
        N = len(self.points)
        i = 0
        while i < N:
            p1, p2, p3 = self.points[i], self.points[(i + 1) % N], self.points[(i + 2) % N]
            if np.cross(p2 - p1, p3 - p2) == 0:
                self.points.pop((i + 1) % N)
                N -= 1
                print(f"WARNING: Collinear points {p1}, {p2}, {p3} found. Removing point {p2} from the polygon.")
            else:
                i += 1

        # Ensure that the polygon has at least 3 non-collinear points
        if N < 3:
            raise ValueError("A polygon must have at least 3 non-collinear points.")
            
        # Ensure the polygon is not self-intersecting
        if not ShapelyPolygon([point.tolist() for point in self.points]).is_simple:
            raise ValueError("Polygon is self-intersecting. Ensure all points were provided in the correct order.")
        
        # Ensure the polygon is convex - TODO: Implement non-convex Polygons (hard)
        if not self._is_convex():
            raise ValueError("Polygon is not convex. Ensure all points are in clockwise or counter-clockwise order.")
        
        # Define equations a*x + b*y <= c (same as A_i*x <= b_i with x = (x, y)) for each edge of the polygon
        self._center = np.mean(self.points, axis=0)
        for index, point in enumerate(self.points):
            next_point = self.points[(index + 1) % N]
            a = next_point[1] - point[1]
            b = point[0] - next_point[0]
            c = a*point[0] + b*point[1]
            if a*self._center[0] + b*self._center[1] < c:
                self._set_equations.append((a, b, c))
            elif a*self._center[0] + b*self._center[1] > c:
                self._set_equations.append((-a, -b, -c))
        
    def _is_convex(self) -> bool:
        """
        Check if the polygon is convex. A polygon is convex if for every pair of points in the polygon, the line segment connecting them lies entirely inside the polygon.
        """
        N = len(self.points)
        p1, p2, p3 = self.points[0], self.points[1], self.points[2]
        direction = np.cross(p2 - p1, p3 - p2) # z-component of cross product
        for i in range(len(self.points) - 1):
            p1, p2, p3 = self.points[i + 1], self.points[(i + 2) % N], self.points[(i + 3) % N]
            if direction*np.cross(p2 - p1, p3 - p2) < 0:
                return False
            
        return True
            
if __name__ == "__main__":
    s1 = Polygon(np.array([(0, 0), (0.5, 0), (1, 0), (1, 0.5), (1, 1), (0, 1)]))
    s2 = Polygon([(0, 0), (0, 1), (1, 1), (1, 0)])
    c1 = Circle(np.array([0, 0]), 1)
    c2 = Circle([0, 0], 1)
    print(s1._set_equations)
    print(s2._set_equations)
    # s3 = Polygon([(0, 1), (1, 0), (0, -1), (-1, 0)])
    pass