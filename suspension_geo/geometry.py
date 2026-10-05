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


    def move(self,new_pos):

        self.pos = np.asarray(new_pos,dtype=float)


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

class RigidBody2D:
    """
    2D rigid body
    """

    def __init__(
        self,
        points,
        reference_pair=None,
        name="RigidBody"
    ):

        self.name = name
        self.points = points


        # 用第一個點當reference
        self.origin = points[0].pos.copy()


        self.initial_points = {
            p:p.pos.copy()
            for p in points
        }


        self.local_points = {}

        for p in points:

            self.local_points[p] = (
                p.pos - self.origin
            )


        # 定義姿態參考兩點
        self.reference_pair = reference_pair



    @staticmethod
    def rotation_matrix(theta):

        c=np.cos(theta)
        s=np.sin(theta)

        return np.array(
            [
                [c,-s],
                [s,c]
            ]
        )


    def update_from_two_points(
        self,
        p1_new,
        p2_new
    ):
        """
        使用兩個點決定rigid body姿態

        p1_new:
            第一個已知位置

        p2_new:
            第二個已知位置
        """


        p1,p2=self.reference_pair


        # 初始方向
        v0 = (
            self.initial_points[p2]
            -
            self.initial_points[p1]
        )


        # 新方向
        v1 = p2_new-p1_new


        theta0=np.arctan2(
            v0[1],
            v0[0]
        )


        theta1=np.arctan2(
            v1[1],
            v1[0]
        )


        dtheta=theta1-theta0


        R=self.rotation_matrix(dtheta)



        # 更新所有點

        for p in self.points:

            local = (
                self.initial_points[p]
                -
                self.initial_points[p1]
            )


            new_pos = (
                p1_new
                +
                R@local
            )


            p.move(new_pos)



    def mirror_x(self):

        mirrored_points=[
            p.mirror_x()
            for p in self.points
        ]

        return RigidBody2D(
            mirrored_points,
            self.reference_pair,
            self.name+"_mirror"
        )


    @property
    def center(self):

        return np.mean(
            [
                p.pos
                for p in self.points
            ],
            axis=0
        )


    def __repr__(self):

        return (
            f"{self.name}\n"
            f"points:{len(self.points)}\n"
            f"center:{self.center}"
        )
    

class Polygon:
    def __init__(self, points, name="Polygon"):
        self.points = points
        self.name = name

class Wheel(Polygon):
    pass

class Body(Polygon):
    pass
