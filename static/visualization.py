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

        # 直接使用靜態幾何座標建立 Polygon。
        points = np.array([point.pos for point in polygon.points])
        self.patch = patches.Polygon(
            points,
            closed=True,
            **kwargs
        )

        ax.add_patch(self.patch)


