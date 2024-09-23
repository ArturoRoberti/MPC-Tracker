import random
import math

# Helper class for a circle
class Circle:
    def __init__(self, center, radius):
        self.center = center
        self.radius = radius

    def contains(self, point):
        """Check if a point is inside the circle"""
        return distance(self.center, point) <= self.radius + 1e-9  # Add small tolerance to handle precision errors

# Helper function to compute the distance between two points
def distance(p1, p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

# Trivial cases when 0, 1, 2, or 3 points define a circle
def make_circle(points):
    if len(points) == 0:
        return Circle((0, 0), 0)  # Empty circle
    elif len(points) == 1:
        return Circle(points[0], 0)  # Single point
    elif len(points) == 2:
        return circle_from_two_points(points[0], points[1])  # Two points define a circle with diameter as the line
    elif len(points) == 3:
        return circle_from_three_points(points[0], points[1], points[2])  # Three points define a circumcircle

def circle_from_two_points(p1, p2):
    center = ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2)
    radius = distance(p1, p2) / 2
    return Circle(center, radius)

def circle_from_three_points(p1, p2, p3):
    # Calculate the circumcenter of the triangle formed by p1, p2, and p3
    ax, ay = p1
    bx, by = p2
    cx, cy = p3
    d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if d == 0:
        return None  # Degenerate case (collinear points)
    
    ux = ((ax**2 + ay**2) * (by - cy) + (bx**2 + by**2) * (cy - ay) + (cx**2 + cy**2) * (ay - by)) / d
    uy = ((ax**2 + ay**2) * (cx - bx) + (bx**2 + by**2) * (ax - cx) + (cx**2 + cy**2) * (bx - ax)) / d
    center = (ux, uy)
    radius = distance(center, p1)
    return Circle(center, radius)

# Welzl's algorithm
def welzl(P, R):
    if len(P) == 0 or len(R) == 3:
        return make_circle(R)

    # Choose a random point p from P
    p = P.pop()
    
    # Get the minimum enclosing circle without point p
    D = welzl(P, R)
    
    # If p is inside D, return D
    if D.contains(p):
        return D
    
    # Otherwise, p must be on the boundary of the new circle
    return welzl(P, R + [p])

# Main function to find the minimum enclosing circle
def find_minimum_enclosing_circle(points):
    # Randomly shuffle the points
    P = points.copy()
    random.shuffle(P)
    # Use Welzl's algorithm
    return welzl(P, [])

# Example usage
if __name__ == "__main__":
    # Example set of points
    points = [(0, 0), (4, 0), (2, 4), (1, 1), (2, 2)]

    # Find the minimum enclosing circle
    circle = find_minimum_enclosing_circle(points)
    
    # Output the result
    print(f"Center: {circle.center}")
    print(f"Radius: {circle.radius}")
