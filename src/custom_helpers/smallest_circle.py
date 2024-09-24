import random
import numpy as np
from custom_helpers.helper_classes import Circle
# import matplotlib.pyplot as plt
from typing import List

def smallest_circle(points: List[np.ndarray]) -> Circle:
    # Check input type and size
    if not isinstance(points, list):
        raise TypeError("Input points must be a list")
    if len(points) == 0:
        raise ValueError("Input points must not be an empty list")
    for index, point in enumerate(points):
        if not isinstance(point, np.ndarray):
            if not (isinstance(point, tuple) or isinstance(point, list)) or len(point) != 2:
                raise ValueError("Each point must be a tuple or list of length 2")
            else:
                points[index] = np.array(point)

    # Trivial cases when 0, 1, 2, or 3 points define a circle
    def _make_circle(points: list) -> Circle:
        def _circle_from_two_points(p1: np.ndarray, p2: np.ndarray) -> Circle:
            center = (p1 + p2)/2
            radius = np.linalg.norm(p1 - p2) / 2
            return Circle(center, radius)
        
        def _circle_from_three_points(p1: np.ndarray, p2: np.ndarray, p3: np.ndarray) -> Circle:
            # Calculate the circumcenter of the triangle formed by p1, p2, and p3
            ax, ay = p1
            bx, by = p2
            cx, cy = p3
            d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
            if d == 0:
                return None  # Degenerate case (collinear points)
            
            ux = ((ax**2 + ay**2) * (by - cy) + (bx**2 + by**2) * (cy - ay) + (cx**2 + cy**2) * (ay - by)) / d
            uy = ((ax**2 + ay**2) * (cx - bx) + (bx**2 + by**2) * (ax - cx) + (cx**2 + cy**2) * (bx - ax)) / d
            center = np.array([ux, uy])
            radius = np.linalg.norm(center - p1)
            return Circle(center, radius)
    
        if len(points) == 0:
            return Circle(np.array([0, 0]), 0)  # Empty circle
        elif len(points) == 1:
            return Circle(points[0], 0)  # Single point
        elif len(points) == 2:
            return _circle_from_two_points(points[0], points[1])  # Two points define a circle with diameter as the line
        elif len(points) == 3:
            return _circle_from_three_points(points[0], points[1], points[2])  # Three points define a circumcircle
        
    # Welzl's algorithm
    def _welzl(P: List[np.ndarray], R: List[np.ndarray], n: int) -> Circle:
        if(n == 0 or len(R) == 3):
            return _make_circle(R)

        # Pick a random point
        idx = random.randint(0,n-1)
        p = P[idx]

        # Put the picked point at the end of P
        # since it's more efficient than
        # deleting from the middle of the vector
        P[idx], P[n-1] = P[n-1], P[idx]
        
        # Get the minimum enclosing circle without point p
        D = _welzl(P, R.copy(), n-1)
        
        # If p is inside D, return D
        if D.contains(p):
            return D
        
        # Otherwise, p must be on the boundary of the minimum enclosing circle
        R.append(p)
        return _welzl(P, R.copy(), n-1)
    
    ### Main function to find the minimum enclosing circle ###
    # Randomly shuffle the points
    P = points.copy()
    random.shuffle(P)
    # Use Welzl's algorithm
    return _welzl(P, [], len(P))

    # def plot_results(points, circle):
    # # Plot the points
    # x, y = zip(*points)
    # plt.scatter(x, y, label='Points')

    # # Plot the circle
    # circle_plot = plt.Circle(circle.center, circle.radius, color='b', fill=False, label='Minimum Enclosing Circle')
    # plt.gca().add_patch(circle_plot)

    # # Set the aspect of the plot to be equal
    # plt.gca().set_aspect('equal', adjustable='box')

    # # Add labels and legend
    # plt.xlabel('X')
    # plt.ylabel('Y')
    # plt.legend()
    # plt.title('Minimum Enclosing Circle')

    # # Show the plot
    # plt.show()

if __name__ == "__main__":
    # # Generate 100 random points
    # points = [(random.uniform(-10, 10), random.uniform(-10, 10)) for _ in range(100)]

    # # points = [(0, 0), (4, 0), (2, 4), (2, 2)]

    # # Find the minimum enclosing circle
    # circle = smallest_circle(points)

    # # Output the result
    # print(f"Center: {circle.center}")
    # print(f"Radius: {circle.radius}")

    # # # Plot the results
    # # plot_results(points, circle)
    pass