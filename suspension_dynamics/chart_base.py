"""Canvas chart implementation reused locally from VehicleLab/vehicle_lab/charts.py."""
"""Offline Tk canvas charts with pan/zoom, SVG and optional Pillow PNG export."""
import html
import math
import tkinter as tk
from tkinter import ttk
from pathlib import Path

COLORS=("#38bdf8","#fbbf24","#a78bfa","#34d399","#fb7185","#fb923c")
BG="#0f172a"
SEGMENT_STYLES={
    "upper":("#fbbf24",()),"lower":("#38bdf8",()),"upright":("#a78bfa",()),
    "rod":("#fb923c",()),"wheel":("#d1d5db",()),"chassis":("#34d399",()),"ground":("#64748b",()),
    "projection_upper":("#fbbf24",(8,3,2,3)),"projection_lower":("#38bdf8",(8,3,2,3)),
    "contact_line":("#2dd4bf",(2,4)),"kingpin":("#fb7185",(7,4))}
SEGMENT_LABELS={"projection_upper":"Upper projection","projection_lower":"Lower projection",
                "contact_line":"Contact to IC","kingpin":"Kingpin"}


class Chart(ttk.Frame):
    def __init__(self,master):
        super().__init__(master)
        self.canvas=tk.Canvas(self,bg=BG,highlightthickness=0,width=800,height=470)
        self.canvas.pack(fill="both",expand=True)
        self.spec=None; self.primitives=[]; self.view=None; self.drag=None
        self.azimuth=-.65; self.elevation=.5
        self.canvas.bind("<Configure>",lambda event:self.draw())
        self.canvas.bind("<MouseWheel>",self.zoom)
        self.canvas.bind("<Button-1>",lambda e:setattr(self,"drag",(e.x,e.y)))
        self.canvas.bind("<B1-Motion>",self.pan)
        self.canvas.bind("<Double-Button-1>",lambda e:self.reset())
        self.canvas.bind("<Motion>",self.hover)

    def set(self,spec):
        self.spec=spec; self.view=None; self.draw()

    def reset(self):
        self.view=None; self.draw()

    def line(self,coords,color,width=1,dash=()):
        self.canvas.create_line(*coords,fill=color,width=width,dash=dash)
        self.primitives.append(("line",list(coords),color,width,tuple(dash)))

    def text(self,x,y,text,color="#cbd5e1",size=10,anchor="center"):
        self.canvas.create_text(x,y,text=str(text),fill=color,font=("Microsoft JhengHei UI",size),anchor=anchor)
        self.primitives.append(("text",[x,y],color,size,str(text),anchor))

    def project(self,point,projection):
        x,y,z=point
        if projection=="Front": return -y,z
        if projection=="Rear": return y,z
        if projection=="Side": return x,z
        if projection=="Top": return -y,x
        ca,sa=math.cos(self.azimuth),math.sin(self.azimuth)
        ce,se=math.cos(self.elevation),math.sin(self.elevation)
        return x*ca-y*sa, z*ce-(x*sa+y*ca)*se

    def draw(self):
        self.canvas.delete("all"); self.primitives=[]
        width=max(self.canvas.winfo_width(),300); height=max(self.canvas.winfo_height(),220)
        spec=self.spec
        if not spec:
            self.text(width/2,height/2,"設定參數後按「計算」",size=15)
            return
        series=spec.get("series",[])
        overlays=[]
        if "segments" in spec:
            projection=spec.get("projection","Iso")
            def projected(segments):
                result=[]
                for a,b,kind in segments:
                    color,dash=SEGMENT_STYLES[kind]
                    pa,pb=self.project(a,projection),self.project(b,projection)
                    result.append({"name":SEGMENT_LABELS.get(kind,kind),"x":[pa[0],pb[0]],
                                   "y":[pa[1],pb[1]],"color":color,"dash":dash})
                return result
            series=projected(spec["segments"])
            overlays=projected(spec.get("overlays",[]))
        # Construction intersections can be hundreds of metres away. Fit the
        # physical vehicle only, then clip construction lines to its viewport.
        pairs=[(x,y) for s in series for x,y in zip(s["x"],s["y"]) if self.finite(x) and self.finite(y)]
        if not pairs:
            self.text(width/2,height/2,"沒有可繪製的數值（可能為飽和或幾何不可達）")
            return
        legend=[];seen=set()
        for index,s in enumerate(series+overlays):
            name=s.get("name","")
            if name and name not in seen:
                seen.add(name)
                legend.append((name,s.get("color",COLORS[index%len(COLORS)]),s.get("dash",())))
        left,right,bottom=85,width-35,height-67
        legend_items=[];xpos=left;legend_y=45
        # Keep geometry's plotting rectangle fixed when display layers change.
        # Otherwise an extra legend row changes the pixel aspect of a saved view.
        legend_slots=[(SEGMENT_LABELS.get(kind,kind),color,dash)
                      for kind,(color,dash) in SEGMENT_STYLES.items()] if "segments" in spec else legend
        hidden_legend=0
        max_legend_y=45+20*max(0,int((height-240)/40))
        for label,color,dash in legend_slots:
            visible=label in seen
            max_chars=max(1,min(25,int((right-left-36)/6.5)))
            label=label[:max_chars]
            item_width=max(90,len(label)*6.5+36)
            if xpos>left and xpos+item_width>right:
                xpos=left;legend_y+=20
            if legend_y>max_legend_y:
                if visible:hidden_legend+=1
                continue
            if visible:legend_items.append((xpos,legend_y,label,color,dash))
            xpos+=item_width
        legend_y=min(legend_y,max_legend_y)
        top=64+max(0,legend_y-45)
        pw,ph=max(right-left,1),max(bottom-top,1)
        if self.view is None:
            xs,ys=zip(*pairs); xmin,xmax=min(xs),max(xs); ymin,ymax=min(ys),max(ys)
            dx=xmax-xmin or max(abs(xmax)*.2,1); dy=ymax-ymin or max(abs(ymax)*.2,1)
            xmin-=dx*.06; xmax+=dx*.06; ymin-=dy*.12; ymax+=dy*.12
            if spec.get("equal") or "segments" in spec:
                ratio=pw/ph
                if (xmax-xmin)/(ymax-ymin)<ratio:
                    mid=(xmin+xmax)/2; span=(ymax-ymin)*ratio; xmin,xmax=mid-span/2,mid+span/2
                else:
                    mid=(ymin+ymax)/2; span=(xmax-xmin)/ratio; ymin,ymax=mid-span/2,mid+span/2
            self.view=[xmin,xmax,ymin,ymax]
        xmin,xmax,ymin,ymax=self.view
        self.bounds=(left,top,right,bottom)
        sx=lambda x:left+(x-xmin)/(xmax-xmin)*pw
        sy=lambda y:bottom-(y-ymin)/(ymax-ymin)*ph
        for i in range(6):
            x=left+pw*i/5; y=top+ph*i/5
            self.line([x,top,x,bottom],"#253247")
            self.line([left,y,right,y],"#253247")
            self.text(x,bottom+17,f"{xmin+(xmax-xmin)*i/5:.4g}",size=9)
            self.text(left-10,y,f"{ymax-(ymax-ymin)*i/5:.4g}",anchor="e",size=9)
        if ymin<0<ymax: self.line([left,sy(0),right,sy(0)],"#64748b")
        if xmin<0<xmax: self.line([sx(0),top,sx(0),bottom],"#64748b")
        self.text(left,22,spec.get("title",""),"#f8fafc",14,"w")
        self.text((left+right)/2,height-19,spec.get("xlabel",""),size=10)
        self.text(10,42,spec.get("ylabel",""),"#94a3b8",9,"w")
        for index,s in enumerate(series+overlays):
            color=s.get("color",COLORS[index%len(COLORS)])
            last=None; path=[]
            def flush():
                if len(path)>=4:self.line(path,color,2,s.get("dash",()))
                path.clear()
            for x,y in zip(s["x"],s["y"]):
                if not self.finite(x) or not self.finite(y): flush(); last=None; continue
                point=(sx(x),sy(y))
                if last is not None:
                    clipped=self.clip(last,point,left,top,right,bottom)
                    if clipped:
                        if path and (abs(path[-2]-clipped[0])>1e-6 or abs(path[-1]-clipped[1])>1e-6):flush()
                        if not path:path.extend(clipped[:2])
                        path.extend(clipped[2:])
                    else:flush()
                last=point
            flush()
        for xpos,y,label,color,dash in legend_items:
            self.line([xpos,y,xpos+17,y],color,3,dash)
            self.text(xpos+23,y,label,color,9,"w")
        if hidden_legend:self.text(right,22,f"+{hidden_legend} curves (see selection)","#94a3b8",9,"e")
        self.text(right,bottom+38,"滾輪縮放 · 拖曳平移 / 立體旋轉 · 雙擊重設",color="#64748b",size=8,anchor="e")

    @staticmethod
    def finite(x): return isinstance(x,(float,int)) and math.isfinite(x)

    @staticmethod
    def clip(a,b,l,t,r,bot):
        x,y=a; dx=b[0]-x; dy=b[1]-y; low,high=0.,1.
        for p,q in ((-dx,x-l),(dx,r-x),(-dy,y-t),(dy,bot-y)):
            if abs(p)<1e-12:
                if q<0: return None
            elif p<0: low=max(low,q/p)
            else: high=min(high,q/p)
            if low>high: return None
        return [x+low*dx,y+low*dy,x+high*dx,y+high*dy]

    def zoom(self,event):
        if self.view is None: return
        factor=.85 if event.delta>0 else 1/.85
        xmin,xmax,ymin,ymax=self.view
        l,t,r,b=self.bounds
        px=max(0,min(1,(event.x-l)/max(r-l,1))); py=max(0,min(1,(b-event.y)/max(b-t,1)))
        cx=xmin+(xmax-xmin)*px; cy=ymin+(ymax-ymin)*py
        self.view=[cx+(xmin-cx)*factor,cx+(xmax-cx)*factor,cy+(ymin-cy)*factor,cy+(ymax-cy)*factor]
        self.draw()

    def pan(self,event):
        if self.drag is None or self.view is None: return
        dx,dy=event.x-self.drag[0],event.y-self.drag[1]
        self.drag=(event.x,event.y)
        if self.spec and self.spec.get("projection")=="Iso":
            self.azimuth+=dx*.01; self.elevation=max(-1.3,min(1.3,self.elevation+dy*.01)); self.view=None
        else:
            xmin,xmax,ymin,ymax=self.view; l,t,r,b=self.bounds
            sx=dx*(xmax-xmin)/max(r-l,1); sy=dy*(ymax-ymin)/max(b-t,1)
            self.view=[xmin-sx,xmax-sx,ymin+sy,ymax+sy]
        self.draw()

    def hover(self,event):
        self.canvas.delete("cursor")
        if self.view is None or not hasattr(self,"bounds"): return
        l,t,r,b=self.bounds
        if not l<event.x<r or not t<event.y<b: return
        xmin,xmax,ymin,ymax=self.view
        x=xmin+(event.x-l)/(r-l)*(xmax-xmin)
        y=ymax-(event.y-t)/(b-t)*(ymax-ymin)
        self.canvas.create_text(r-5,t+14,text=f"x={x:.5g}   y={y:.5g}",fill="#f8fafc",anchor="e",font=("Segoe UI",9),tags="cursor")

    def svg(self):
        width=max(self.canvas.winfo_width(),300); height=max(self.canvas.winfo_height(),220)
        elements=[f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
                  f'<rect width="100%" height="100%" fill="{BG}"/>']
        for item in self.primitives:
            kind,xy,color,size,*rest=item
            if kind=="line":
                coords=" ".join(f"{xy[i]:.2f},{xy[i+1]:.2f}" for i in range(0,len(xy),2))
                dash=rest[0] if rest else ()
                pattern=' stroke-dasharray="'+" ".join(str(d) for d in dash)+'"' if dash else ""
                elements.append(f'<polyline points="{coords}" fill="none" stroke="{color}" stroke-width="{size}"{pattern}/>')
            elif kind=="rect":
                elements.append(f'<rect x="{xy[0]:.2f}" y="{xy[1]:.2f}" width="{xy[2]-xy[0]:.2f}" height="{xy[3]-xy[1]:.2f}" fill="{color}" stroke="{rest[0]}" stroke-width="{size}"/>')
            else:
                content,anchor=rest
                alignment="end" if anchor=="e" else "start" if anchor=="w" else "middle"
                elements.append(f'<text x="{xy[0]:.2f}" y="{xy[1]+size*.4:.2f}" fill="{color}" font-size="{size*1.3}" font-family="Microsoft JhengHei UI, sans-serif" text-anchor="{alignment}">{html.escape(content)}</text>')
        return "\n".join(elements+["</svg>"])

    def png(self,path):
        try:
            from PIL import Image,ImageDraw,ImageFont
        except ImportError as exc:
            raise ValueError("PNG 需要選用套件 Pillow：python -m pip install Pillow。SVG 不需套件。") from exc
        factor=2
        width=max(self.canvas.winfo_width(),300); height=max(self.canvas.winfo_height(),220)
        image=Image.new("RGB",(width*factor,height*factor),BG); draw=ImageDraw.Draw(image)
        font_path=Path("C:/Windows/Fonts/msjh.ttc")
        for item in self.primitives:
            kind,xy,color,size,*rest=item
            scaled=[x*factor for x in xy]
            if kind=="line":
                dash=rest[0] if rest else ()
                if not dash:draw.line(scaled,fill=color,width=max(1,int(size*factor)))
                else:
                    for a,b in self.dashed_segments(scaled,[d*factor for d in dash]):
                        draw.line([*a,*b],fill=color,width=max(1,int(size*factor)))
            elif kind=="rect":
                draw.rectangle(scaled,fill=color,outline=rest[0],width=max(1,int(size*factor)))
            else:
                text,anchor=rest
                font=ImageFont.truetype(str(font_path),int(size*1.3*factor)) if font_path.exists() else ImageFont.load_default()
                align="rm" if anchor=="e" else "lm" if anchor=="w" else "mm"
                draw.text(scaled,text,fill=color,font=font,anchor=align)
        image.save(path)

    @staticmethod
    def dashed_segments(coords,dash):
        """Yield visible dash pieces with continuous phase along a polyline."""
        index=0;remaining=dash[0]
        for offset in range(0,len(coords)-2,2):
            x,y=coords[offset:offset+2];dx=coords[offset+2]-x;dy=coords[offset+3]-y
            length=math.hypot(dx,dy)
            if length<1e-12:continue
            distance=0.
            while distance<length:
                step=min(remaining,length-distance)
                if index%2==0:
                    yield ((x+dx*distance/length,y+dy*distance/length),
                           (x+dx*(distance+step)/length,y+dy*(distance+step)/length))
                distance+=step;remaining-=step
                if remaining<1e-9:index=(index+1)%len(dash);remaining=dash[index]
