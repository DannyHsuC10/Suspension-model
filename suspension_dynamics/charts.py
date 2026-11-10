"""VehicleLab Canvas charts; static curves plus lightweight retained playback."""
import math
import tkinter as tk
from tkinter import ttk

import numpy as np

from .chart_base import Chart, BG, COLORS
from .road_profiles import ISO_ROUGHNESS

WHEEL_RADIUS = .22
WHEEL_WIDTH = .20
BODY_HEIGHT = .85
QUARTER_ROAD_DISPLAY_SPEED = 1.0  # Map recorded time to distance for the animation only.
WHEEL_ITEMS = ("wheel", "wheel_far", *(f"wheel_edge{i}" for i in range(4)))


def wheel_paths(center, projection):
    """Orthographic tire profiles and an axial-Y cylinder for the 3D view."""
    x,y,z=center
    r,w=WHEEL_RADIUS,WHEEL_WIDTH/2
    if projection == "Front":
        return {"wheel":[(x,y-w,z-r),(x,y+w,z-r),(x,y+w,z+r),(x,y-w,z+r),(x,y-w,z-r)]}
    angles=np.linspace(0,2*np.pi,49)
    def circle(offset):
        return [(x+r*math.cos(t),y+offset,z+r*math.sin(t)) for t in angles]
    if projection == "Side":return {"wheel":circle(0.)}
    paths={"wheel":circle(-w),"wheel_far":circle(w)}
    for i,t in enumerate((0.,math.pi/2,math.pi,3*math.pi/2)):
        paths[f"wheel_edge{i}"]=[(x+r*math.cos(t),y-w,z+r*math.sin(t)),(x+r*math.cos(t),y+w,z+r*math.sin(t))]
    return paths


def envelope(x, y, budget=1600):
    """Keep per-bucket extrema, so short wheel-hop spikes remain visible."""
    x, y = np.asarray(x), np.asarray(y)
    if len(x) <= budget: return x.tolist(),y.tolist()
    indices = [0]
    for chunk in np.array_split(np.arange(1,len(x)-1),max(1,budget//2-1)):
        if not len(chunk): continue
        indices.extend(sorted({int(chunk[np.argmin(y[chunk])]),int(chunk[np.argmax(y[chunk])])}))
    indices.append(len(x)-1)
    return x[indices].tolist(),y[indices].tolist()


class ResponseChart(Chart):
    def __init__(self, master):
        super().__init__(master)
        self.playback_time = None

    def set(self,spec):
        self.playback_time = None
        if spec:
            series=[]
            for s in spec.get("series",[]):
                x,y=envelope(s["x"],s["y"])
                series.append(dict(s,x=x,y=y))
            spec = dict(spec,series=series)
        super().set(spec)

    def draw(self):
        super().draw()
        if self.playback_time is not None: self.cursor(self.playback_time)

    def cursor(self,time):
        self.playback_time = time
        self.canvas.delete("playback")
        if not self.view or not hasattr(self,"bounds"): return
        lo,hi,_,_ = self.view
        l,t,r,b = self.bounds
        x = l+(time-lo)/(hi-lo)*(r-l)
        if l <= x <= r:
            self.canvas.create_line(x,t,x,b,fill="#f8fafc",dash=(3,3),tags="playback")
            self.canvas.create_text(x+5,b-8,text=f"{time:.3f} s",anchor="sw",fill="#f8fafc",font=("Segoe UI",9),tags="playback")


class MotionView(ttk.Frame):
    """Retain a small number of Canvas objects; never rebuild curves per frame."""
    def __init__(self,master):
        super().__init__(master)
        self.canvas = tk.Canvas(self,bg=BG,highlightthickness=0,height=230)
        self.canvas.pack(fill="both",expand=True)
        self.result = None
        self.motion_peak = 0.
        self.index = 0
        self.gain = 1.
        self.azimuth, self.elevation = -.7,.5
        self.projection = "Iso"
        self.items = []
        self.drag = None
        self.zoom_scale = 1.
        self.pan_x = self.pan_y = 0.
        self.canvas.bind("<Configure>",lambda e:self.update_frame(self.index))
        self.canvas.bind("<Button-1>",lambda e:setattr(self,"drag",(e.x,e.y)))
        self.canvas.bind("<B1-Motion>",self.pan)
        self.canvas.bind("<MouseWheel>",self.zoom)
        self.canvas.bind("<Double-Button-1>",lambda e:self.reset())
        self.reset_items()

    def reset_items(self):
        self.canvas.delete("all")
        self.road_fill = None
        if self.result is not None and self.result.family == "quarter":
            self.road_fill = self.canvas.create_polygon(0, 0, 0, 0, fill="#422b20", state="hidden")
        self.header = self.canvas.create_text(14,14,anchor="nw",fill="#e2e8f0",font=("Microsoft JhengHei UI",10))
        self.hint = self.canvas.create_text(14,40,anchor="nw",fill="#94a3b8",font=("Microsoft JhengHei UI",9))
        self.body = self.canvas.create_line(0,0,0,0,fill="#34d399",width=5)
        self.items=[]
        for i in range(4):
            self.items.append({"spring":self.canvas.create_line(0,0,0,0,fill=COLORS[i],width=2),
                               "damper":self.canvas.create_line(0,0,0,0,fill="#94a3b8",width=4),
                               **{name:self.canvas.create_line(0,0,0,0,fill="#cbd5e1",width=3) for name in WHEEL_ITEMS},
                               "ground":self.canvas.create_line(0,0,0,0,fill="#fb923c",width=3),
                               "label":self.canvas.create_text(0,0,text="",fill=COLORS[i],font=("Segoe UI",9))})

    def set_result(self,result):
        self.result = result
        self.motion_peak=max((float(np.max(np.abs(v)))/1000 for k,v in result.signals.items() if k.startswith(("Body ","Wheel ","Road ")) and result.units[k]=="mm"),default=0.) if result is not None else 0.
        self.index = 0
        self.zoom_scale=1.;self.pan_x=self.pan_y=0.
        self.reset_items()
        self.update_frame(0)

    def project(self,point):
        x,y,z=point
        if self.projection == "Front": return -y,z
        if self.projection == "Side": return x,z
        ca,sa=math.cos(self.azimuth),math.sin(self.azimuth)
        ce,se=math.cos(self.elevation),math.sin(self.elevation)
        return x*ca-y*sa,z*ce-(x*sa+y*ca)*se

    def to_canvas(self,point):
        x,y=self.project(point)
        return self.origin[0]+x*self.pixels_per_metre,self.origin[1]-y*self.pixels_per_metre

    def fit_geometry(self, positions, road_profile=False):
        """One pixels/metre scale for both axes, with a stable recording-wide fit."""
        motion=self.motion_peak*self.gain
        margin=max(WHEEL_RADIUS,.28)
        longitudinal_margin = max(margin, .6) if road_profile else margin
        xs=[p[0] for p in positions];ys=[p[1] for p in positions]
        bounds=((min(xs)-longitudinal_margin,max(xs)+longitudinal_margin),(min(ys)-margin,max(ys)+margin),
                (-motion,BODY_HEIGHT+motion))
        points=[self.project((x,y,z)) for x in bounds[0] for y in bounds[1] for z in bounds[2]]
        px,py=zip(*points)
        xmin,xmax,ymin,ymax=min(px),max(px),min(py),max(py)
        width=max(1,self.canvas.winfo_width());height=max(1,self.canvas.winfo_height())
        left,right,top,bottom=40.,40.,64.,38.
        usable_width=max(1.,width-left-right);usable_height=max(1.,height-top-bottom)
        self.pixels_per_metre=min(usable_width/max(xmax-xmin,1e-9),usable_height/max(ymax-ymin,1e-9))*self.zoom_scale
        self.origin=(left+usable_width/2-(xmin+xmax)/2*self.pixels_per_metre+self.pan_x,
                     top+usable_height/2+(ymin+ymax)/2*self.pixels_per_metre+self.pan_y)

    def draw_quarter_road(self, outline):
        result = self.result
        width = self.canvas.winfo_width()
        wheel_x, datum_y = self.to_canvas((0, 0, 0))
        xs = np.linspace(0, width, min(width + 1, 401))
        if 0 < wheel_x < width:
            # Keep the input directly below the wheel exact, even between pixels.
            xs = np.insert(xs, np.searchsorted(xs, wheel_x), wheel_x)
        options = result.project["analysis"]["quarter"]
        speed = options["speed"] if options["scenario"] in ISO_ROUGHNESS else QUARTER_ROAD_DISPLAY_SPEED
        offsets = (xs - wheel_x) / (self.pixels_per_metre * speed)
        heights = np.interp(result.time[self.index] + offsets, result.time, result.signals["Road Q"])
        ys = datum_y - heights * (self.gain * self.pixels_per_metre / 1000)
        surface = np.column_stack((xs, ys)).ravel().tolist()
        bottom = max(self.canvas.winfo_height(), float(ys.max()))
        self.canvas.coords(outline, *surface)
        self.canvas.coords(self.road_fill, *surface, width, bottom, 0, bottom)

    def update_frame(self,index):
        r=self.result
        if r is None:
            self.canvas.itemconfigure(self.header,text="計算後即可播放模型動畫")
            return
        self.index=max(0,min(int(index),len(r.time)-1));index=self.index
        p=r.project["parameters"][r.family]
        if r.family == "full":
            labels=("FL","FR","RL","RR")
            positions=[(p["lf"],p["tf"]/2),(p["lf"],-p["tf"]/2),(-p["lr"],p["tr"]/2),(-p["lr"],-p["tr"]/2)]
        elif r.family == "LR": labels=("L","R");positions=[(0,p["track"]/2),(0,-p["track"]/2)]
        elif r.family == "FR": labels=("F","R");positions=[(p["lf"],0),(-p["lr"],0)]
        else: labels=("Q",);positions=[(0,0)]
        def value(name,default=0.): return r.signals[name][index]/1000 if name in r.signals else default
        coords=[]
        for label,(x,y) in zip(labels,positions):
            road=value("Road "+label)
            wheel=value("Wheel "+label,road)
            body=value("Body "+label)
            coords.append(((x,y,BODY_HEIGHT+body*self.gain),(x,y,WHEEL_RADIUS+wheel*self.gain),(x,y,road*self.gain)))
        show_road = self.road_fill is not None and self.projection == "Side" and "Road Q" in r.signals
        if self.road_fill is not None:
            self.canvas.itemconfigure(self.road_fill, state="normal" if show_road else "hidden")
        self.fit_geometry(positions, road_profile=show_road)
        pixel=self.to_canvas
        def path(item,points): self.canvas.coords(item,*[v for point in points for v in pixel(point)])
        body_points=[pixel(c[0]) for c in coords]
        if len(coords)==4:
            body_points=[body_points[i] for i in (0,1,3,2,0)]
        if len(coords)==1:
            a=coords[0][0];dx=.28
            body_points=[pixel((a[0]-dx,a[1],a[2])),pixel((a[0]+dx,a[1],a[2]))]
            if self.projection == "Front":body_points=[pixel((a[0],a[1]-dx,a[2])),pixel((a[0],a[1]+dx,a[2]))]
        self.canvas.coords(self.body,*[v for point in body_points for v in point])
        for i,item in enumerate(self.items):
            active=i<len(coords)
            for identity in item.values():self.canvas.itemconfigure(identity,state="normal" if active else "hidden")
            if not active:continue
            upper,wheel,ground=coords[i]
            a,b=pixel(upper),pixel(wheel)
            dx,dy=b[0]-a[0],b[1]-a[1]
            length=max(1,math.hypot(dx,dy));nx,ny=-dy/length,dx/length
            spring=[a]
            for j in range(1,12):
                f=j/12; offset=(1 if j%2 else -1)*7
                spring.append((a[0]+dx*f+nx*offset,a[1]+dy*f+ny*offset))
            spring.append(b)
            self.canvas.coords(item["spring"],*[v for point in spring for v in point])
            self.canvas.coords(item["damper"],a[0]+nx*15,a[1]+ny*15,b[0]+nx*15,b[1]+ny*15)
            profiles=wheel_paths(wheel,self.projection)
            for name in WHEEL_ITEMS:
                self.canvas.itemconfigure(item[name],state="normal" if name in profiles else "hidden")
                if name in profiles:path(item[name],profiles[name])
            x,y,z=ground
            if show_road:
                self.draw_quarter_road(item["ground"])
            else:
                path(item["ground"],[(x,y-.18,z),(x,y+.18,z)] if self.projection=="Front" else [(x-.28,y,z),(x+.28,y,z)])
            self.canvas.coords(item["label"],b[0],b[1]+30)
            self.canvas.itemconfigure(item["label"],text=f"{labels[i]}  {r.signals.get('Body '+labels[i],np.zeros(1))[index] if 'Body '+labels[i] in r.signals else 0:.2f} mm")
        geometry_label = "舊資料示意 1:1" if r.legacy else "幾何比例 1:1"
        self.canvas.itemconfigure(self.header, text=f"{r.info.get('model_label', r.model)} · {geometry_label}")
        road_label = f"路面 {r.signals['Road Q'][index]:.2f} mm" if show_road else "路面"
        self.canvas.itemconfigure(self.hint, text=f"綠：車身   彩：懸吊   灰：輪位   橘：{road_label}")

    def pan(self,event):
        if self.drag is None:return
        dx,dy=event.x-self.drag[0],event.y-self.drag[1];self.drag=event.x,event.y
        if self.projection=="Iso":
            self.azimuth+=dx*.01;self.elevation=max(-1.3,min(1.3,self.elevation+dy*.01))
        else:self.pan_x+=dx;self.pan_y+=dy
        self.update_frame(self.index)

    def zoom(self,event):
        self.zoom_scale=max(.2,min(8.,self.zoom_scale*(1.1 if event.delta>0 else 1/1.1)))
        self.update_frame(self.index)

    def reset(self):
        self.zoom_scale=1.;self.pan_x=self.pan_y=0.;self.azimuth=-.7;self.elevation=.5
        self.update_frame(self.index)
