import numpy as np
import matplotlib.patches as patches

class PointPlot:

    def __init__(
        self,
        ax,
        point,
        **kwargs
    ):

        self.point = point

        self.marker, = ax.plot(
            point.x,
            point.y,
            marker='o',**kwargs
        )


    def update(self):

        self.marker.set_data(
            [self.point.x],
            [self.point.y]
        )

class LinkPlot:
    
    def __init__(
        self,
        ax,
        link,
        **kwargs
    ):

        self.link = link


        pts=np.vstack(
            (
                link.start.pos,
                link.end.pos
            )
        )


        self.line, = ax.plot(
            pts[:,0],
            pts[:,1],
            **kwargs
        )


    def update(self):

        pts=np.vstack(
            (
                self.link.start.pos,
                self.link.end.pos
            )
        )


        self.line.set_data(
            pts[:,0],
            pts[:,1]
        )


def plot_link(ax, link, **kwargs):
    pts = np.vstack((link.start.pos,link.end.pos))
    line, = ax.plot(pts[:,0],pts[:,1],**kwargs)

    return line

def plot_joint(ax, point, **kwargs):
    marker, = ax.plot(point.x,point.y, marker='o',**kwargs)

    return marker

def annotate_point(ax, point):
    
    ax.annotate(
        point.name,
        xy=(point.x,point.y),
        xytext=(8,8),
        textcoords="offset points",
        fontsize=8,
        color="black")

def annotate_link(ax,link):
    
    mid = link.midpoint
    text = (f"{link.name}\n"f"L={link.length*1000:.1f}mm")
    ax.annotate(text,xy=mid,xytext=(0,10),textcoords="offset points",fontsize=8,ha="center")

def plot_geometry_debug(ax,points,links):

    for p in points:
        ax.plot(p.x,p.y,'ro')
        annotate_point(ax,p)

    for link in links:
        plot_link(ax,link)
        annotate_link(ax,link)

class PolygonPlot:

    def __init__(self, ax, polygon, **kwargs):

        self.polygon = polygon

        # 建立初始 Polygon
        self.patch = patches.Polygon(
            np.zeros((len(polygon.points), 2)),
            closed=True,
            **kwargs
        )

        ax.add_patch(self.patch)

        self.update()


    def update(self):

        points = np.array(
            [
                p.pos
                for p in self.polygon.points
            ]
        )

        self.patch.set_xy(points)

