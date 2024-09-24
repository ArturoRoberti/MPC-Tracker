import numpy as np

# Helper class for a circle
class Circle:
    def __init__(self, center: np.ndarray, radius: float) -> None:
        self.center = center
        self.radius = radius

    def contains(self, point: np.ndarray) -> bool:
        """Check if a point is inside the circle"""
        return np.linalg.norm(self.center - point) <= self.radius