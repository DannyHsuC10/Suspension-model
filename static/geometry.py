import numpy as np

class Point:
    """2D Point"""

    def __init__(self, pos, name="Point"):

        self.name = name
        self.pos = np.asarray(pos, dtype=float)

    @property
    def x(self):
        return self.pos[0]

    @property
    def y(self):
        return self.pos[1]

    def mirror_x(self):

        return Point(
            self.pos*np.array([-1.0,1.0]),
            self.name+"_L"
        )

    def __repr__(self):

        return (
            f"{self.name}: "
            f"({self.x:.3f},{self.y:.3f})"
        )
    
class Link:
    """Generic suspension link"""

    def __init__(self, start, end, name="Link"):
        """
        Parameters
        ----------
        start : Point
        end   : Point
        """
    
        self.name = name
        self.start = start
        self.end = end

    @property
    def vector(self):
        return self.end.pos - self.start.pos

    @property
    def length(self):
        return np.linalg.norm(self.vector)
    
    @property
    def angle(self):

        return np.arctan2(
            self.vector[1],
            self.vector[0]
        )
    
    @property
    def direction(self):
        L = self.length
        if np.isclose(L, 0.0):
            return np.zeros(2)
        return self.vector / L

    @property
    def midpoint(self):
        return 0.5 * (self.start.pos + self.end.pos)

    @property
    def angle(self):
        return np.arctan2(self.vector[1], self.vector[0])

    def __repr__(self):
        return (
            f"{self.name}\n"
            f" start : {self.start.pos}\n"
            f" end   : {self.end.pos}\n"
            f" length: {self.length:.4f} m"
        )
    
    def mirror_x(self):

        return Link(
            self.start.mirror_x(),
            self.end.mirror_x(),
            self.name
        )

    

class Polygon:
    def __init__(self, points, name="Polygon"):
        self.points = points
        self.name = name

class Wheel(Polygon):
    pass

class Body(Polygon):
    pass
