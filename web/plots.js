const colors = ["#62dec8", "#83b5ff", "#ffb56e", "#eda3df", "#e4d989", "#a5aafa", "#a4d38a"];
const NS = "http://www.w3.org/2000/svg";
const background = "#111d31";
const labelColor = "#b9cade";
const gridColor = "#2a3d57";

function svg(parent, tag, attributes = {}, content) {
  const element = document.createElementNS(NS, tag);
  for (const [name, value] of Object.entries(attributes)) element.setAttribute(name, value);
  if (content !== undefined) element.textContent = content;
  parent.append(element);
  return element;
}

function caption(parent, x, y, text, anchor = "middle") {
  return svg(parent, "text", { x, y, fill: labelColor, "font-size": 12, "text-anchor": anchor }, text);
}

export function sampleIndex(time, value) {
  let lo = 0, hi = time.length;
  while (lo < hi) {
    const middle = (lo + hi) >>> 1;
    if (time[middle] <= value) lo = middle + 1;
    else hi = middle;
  }
  return Math.max(0, lo - 1);
}

function compactIndices(values, start, end, budget = 1300) {
  const stride = Math.max(1, Math.ceil((end - start + 1) / (budget / 2)));
  const indices = [start];
  for (let first = start; first <= end; first += stride) {
    let low = first, high = first;
    for (let i = first + 1; i < Math.min(first + stride, end + 1); i++) {
      if (values[i] < values[low]) low = i;
      if (values[i] > values[high]) high = i;
    }
    indices.push(Math.min(low, high));
    if (low !== high) indices.push(Math.max(low, high));
  }
  indices.push(end);
  return indices;
}

const chartBox = { left: 67, top: 29, width: 591, height: 235 };
const formatTick = value => Number(value.toPrecision(4)).toString();

export class ResponsePlot {
  constructor(element) {
    this.element = element;
    this.series = [];
    this.time = 0;
    this.element.setAttribute("font-family", "system-ui, sans-serif");
    this.element.addEventListener("wheel", event => this.zoom(event), { passive: false });
    this.element.addEventListener("dblclick", () => this.reset());
    this.element.addEventListener("pointerdown", event => {
      if (!this.series.length || event.button !== 0) return;
      this.element.setPointerCapture(event.pointerId);
      this.drag = { x: this.xAt(event), view: [...this.view] };
    });
    this.element.addEventListener("pointermove", event => {
      if (!this.drag) return;
      const [start, end] = this.drag.view;
      const delta = (this.drag.x - this.xAt(event)) * (end - start);
      this.setView(start + delta, end + delta);
    });
    for (const event of ["pointerup", "lostpointercapture"]) {
      this.element.addEventListener(event, () => { this.drag = null; });
    }
    this.draw();
  }

  set(series, unit) {
    this.series = series;
    this.unit = unit;
    this.duration = Math.max(0, ...series.map(({ time }) => time.at(-1)));
    this.reset();
  }

  reset() {
    this.view = [0, this.duration || 1];
    this.draw();
  }

  xAt(event) {
    const point = this.element.createSVGPoint();
    point.x = event.clientX;
    point.y = event.clientY;
    const screen = this.element.getScreenCTM();
    return screen ? (point.matrixTransform(screen.inverse()).x - chartBox.left) / chartBox.width : 0;
  }

  zoom(event) {
    if (!this.series.length) return;
    const fraction = this.xAt(event);
    if (fraction < 0 || fraction > 1) return;
    event.preventDefault();
    const [start, end] = this.view;
    const oldWidth = end - start;
    const width = Math.min(this.duration, Math.max(this.duration / 1000,
      oldWidth * (event.deltaY < 0 ? .8 : 1.25)));
    const center = start + oldWidth * fraction;
    this.setView(center - width * fraction, center + width * (1 - fraction));
  }

  setView(start, end) {
    const width = Math.min(end - start, this.duration);
    const left = Math.max(0, Math.min(this.duration - width, start));
    this.view = [left, left + width];
    this.draw();
  }

  draw() {
    const root = this.element;
    root.replaceChildren();
    this.cursor = null;
    svg(root, "rect", { width: 680, height: 310, fill: background });
    const { left, top, width, height } = chartBox;
    if (!this.series.length) {
      for (let i = 0; i <= 6; i++) {
        const x = left + width * i / 6;
        svg(root, "line", { x1: x, x2: x, y1: top, y2: top + height, stroke: gridColor });
      }
      for (let i = 0; i <= 5; i++) {
        const y = top + height * i / 5;
        svg(root, "line", { x1: left, x2: left + width, y1: y, y2: y, stroke: gridColor });
      }
      caption(root, 348, 153, "計算後選擇要顯示的訊號");
      return;
    }
    const [start, end] = this.view;
    const lines = this.series.map(({ name, time, values, color, dash }) => ({
      name, time, values, color, dash,
      indices: compactIndices(values, sampleIndex(time, start), Math.min(time.length - 1, sampleIndex(time, end) + 1)),
    }));
    let low = Infinity, high = -Infinity;
    for (const line of lines) {
      for (const i of line.indices) {
        low = Math.min(low, line.values[i]);
        high = Math.max(high, line.values[i]);
      }
    }
    const pad = low === high ? Math.max(1e-8, Math.abs(low) * .06, .001) : (high - low) * .07;
    low -= pad;
    high += pad;
    const x = time => left + width * (time - start) / (end - start);
    const y = value => top + height * (high - value) / (high - low);
    for (let i = 0; i <= 6; i++) {
      const t = start + (end - start) * i / 6;
      svg(root, "line", { x1: x(t), x2: x(t), y1: top, y2: top + height, stroke: gridColor });
      caption(root, x(t), top + height + 19, formatTick(t));
    }
    for (let i = 0; i <= 5; i++) {
      const v = low + (high - low) * i / 5;
      svg(root, "line", { x1: left, x2: left + width, y1: y(v), y2: y(v), stroke: gridColor });
      caption(root, left - 8, y(v) + 4, formatTick(v), "end");
    }
    caption(root, left, 18, this.unit, "start");
    caption(root, left + width / 2, 304, "時間 (s)");
    const clip = svg(svg(root, "defs"), "clipPath", { id: "response-clip" });
    svg(clip, "rect", { x: left, y: top, width, height });
    const layer = svg(root, "g", { "clip-path": "url(#response-clip)" });
    for (const line of lines) {
      const d = line.indices.map((i, position) =>
        `${position ? "L" : "M"}${x(line.time[i]).toFixed(2)} ${y(line.values[i]).toFixed(2)}`).join(" ");
      const path = svg(layer, "path", { d, fill: "none", stroke: line.color, "stroke-width": 1.7 });
      if (line.dash) path.setAttribute("stroke-dasharray", line.dash);
      svg(path, "title", {}, line.name);
    }
    this.cursor = svg(layer, "line", { y1: top, y2: top + height, stroke: "#eaf1fb", "stroke-dasharray": "4 3" });
    this.setTime(this.time);
  }

  setTime(time) {
    this.time = time;
    if (!this.cursor) return;
    const x = chartBox.left + chartBox.width * (time - this.view[0]) / (this.view[1] - this.view[0]);
    this.cursor.setAttribute("x1", x);
    this.cursor.setAttribute("x2", x);
  }

  exportSvg() {
    const copy = this.element.cloneNode(true);
    const height = 310 + Math.ceil(this.series.length / 2) * 19;
    copy.setAttribute("viewBox", `0 0 680 ${height}`);
    copy.setAttribute("width", "1020");
    copy.setAttribute("height", height * 1.5);
    copy.removeAttribute("id");
    svg(copy, "rect", { y: 310, width: 680, height: height - 310, fill: background });
    for (const [i, { name, color, dash }] of this.series.entries()) {
      const x = 22 + (i % 2) * 332;
      const y = 320 + Math.floor(i / 2) * 19;
      const line = svg(copy, "line", { x1: x, x2: x + 22, y1: y, y2: y, stroke: color, "stroke-width": 2 });
      if (dash) line.setAttribute("stroke-dasharray", dash);
      caption(copy, x + 28, y + 4, name, "start");
    }
    return new XMLSerializer().serializeToString(copy);
  }
}

export function curveColor(index) {
  return colors[index % colors.length];
}

function polyline(parent, points, stroke, width, extra = {}) {
  return svg(parent, "polyline", { points: points.map(point => point.join(",")).join(" "),
    stroke, "stroke-width": width, fill: "none", "stroke-linejoin": "round", ...extra });
}

const radius = .16;
const bodyHeight = .54;
const wheelWidth = .18;

export function drawMotion(root, result, index, view, gain) {
  root.replaceChildren();
  root.setAttribute("font-family", "system-ui, sans-serif");
  svg(root, "rect", { width: 500, height: 300, fill: background });
  if (!result) {
    caption(root, 250, 151, "計算後即可播放模型動畫");
    return;
  }
  const family = result.meta.family;
  const p = result.meta.project.parameters[family];
  const points = {
    quarter: [["Q", 0, 0]],
    LR: [["L", 0, (p.track ?? 1.25) / 2], ["R", 0, -(p.track ?? 1.25) / 2]],
    FR: [["F", p.lf ?? .765, 0], ["R", -(p.lr ?? .765), 0]],
    full: [["FL", p.lf ?? .765, (p.tf ?? 1.25) / 2], ["FR", p.lf ?? .765, -(p.tf ?? 1.25) / 2],
      ["RL", -(p.lr ?? .765), (p.tr ?? 1.25) / 2], ["RR", -(p.lr ?? .765), -(p.tr ?? 1.25) / 2]],
  }[family];
  const projection = (x, y, z) => view === "front" ? [-y, z]
    : view === "iso" ? [x - y * .7, z + x * .22 + y * .35] : [x, z];
  const base = points.flatMap(([, x, y]) => [projection(x, y, 0), projection(x, y, bodyHeight)]);
  const u = base.map(point => point[0]);
  const v = base.map(point => point[1]);
  const span = Math.max(...u) - Math.min(...u) + (family === "quarter" ? 1.8 : .85);
  const height = Math.max(...v) - Math.min(...v) + .6;
  const scale = Math.min(450 / span, 216 / height);
  const midU = (Math.max(...u) + Math.min(...u)) / 2;
  const midV = (Math.max(...v) + Math.min(...v)) / 2;
  const project = (x, y, z) => {
    const [u, v] = projection(x, y, z);
    return [250 + (u - midU) * scale, 153 - (v - midV) * scale];
  };
  const sample = (name, fallback = 0) => (result.signals[name]?.[index] ?? fallback) * gain / 1000;
  const body = [];
  const scrolling = family === "quarter" && result.signals["Road Q"];
  if (scrolling) {
    const opt = result.meta.project.analysis.quarter;
    const speed = opt.scenario.startsWith("iso_") ? opt.speed : 1;
    const time = result.time[index];
    const road = [];
    for (let i = -70; i <= 70; i++) {
      const x = i * .9 / 70;
      const j = sampleIndex(result.time, time + x / speed);
      road.push(project(x, 0, result.signals["Road Q"][j] * gain / 1000));
    }
    polyline(root, road, "#ffb56e", 2);
  }
  for (const [i, [label, x, y]] of points.entries()) {
    const road = sample(`Road ${label}`);
    const wheel = radius + sample(`Wheel ${label}`, result.signals[`Road ${label}`]?.[index]);
    const top = project(x, y, bodyHeight + sample(`Body ${label}`));
    const bottom = project(x, y, wheel);
    body.push(top);
    if (!scrolling) {
      const dx = view === "front" ? 0 : .24;
      const dy = view === "front" ? .24 : 0;
      polyline(root, [project(x - dx, y - dy, road), project(x + dx, y + dy, road)], "#ffb56e", 2);
    }
    const dx = bottom[0] - top[0], dy = bottom[1] - top[1];
    const length = Math.max(1, Math.hypot(dx, dy));
    const spring = [top];
    for (let j = 1; j < 12; j++) {
      const offset = j % 2 ? 5 : -5;
      spring.push([top[0] + dx * j / 12 - dy * offset / length,
        top[1] + dy * j / 12 + dx * offset / length]);
    }
    spring.push(bottom);
    polyline(root, spring, curveColor(i), 2);
    polyline(root, [[top[0] + 12, top[1]], [bottom[0] + 12, bottom[1]]], "#95a8bf", 1.4);
    const stroke = "#91a6be";
    if (view === "front") {
      svg(root, "rect", { x: bottom[0] - wheelWidth * scale / 2, y: bottom[1] - radius * scale,
        width: wheelWidth * scale, height: 2 * radius * scale, fill: "none", stroke, "stroke-width": 2 });
    } else if (view === "iso") {
      const faces = [-wheelWidth / 2, wheelWidth / 2].map(offset => Array.from({ length: 33 }, (_, j) => {
        const angle = j * Math.PI / 16;
        return project(x + radius * Math.cos(angle), y + offset, wheel + radius * Math.sin(angle));
      }));
      for (const face of faces) polyline(root, face, stroke, 1.4);
      for (const i of [0, 8, 16, 24]) polyline(root, faces.map(face => face[i]), stroke, 1.4);
    } else {
      svg(root, "circle", { cx: bottom[0], cy: bottom[1], r: radius * scale, fill: "none", stroke, "stroke-width": 2 });
    }
    caption(root, bottom[0], bottom[1] + radius * scale + 19, label);
  }
  const outline = family === "quarter" ? [[body[0][0] - 50, body[0][1]], [body[0][0] + 50, body[0][1]]]
    : family === "full" ? [0, 1, 3, 2, 0].map(i => body[i]) : body;
  polyline(root, outline, "#62dec8", 5, { "stroke-linecap": "round" });
  caption(root, 15, 24, "幾何比例 1:1", "start");
  caption(root, 250, 284, "綠：車身　彩：懸吊　灰：輪胎　橘：路面");
}
