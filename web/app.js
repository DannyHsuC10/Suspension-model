import { ResponsePlot, curveColor, drawMotion, sampleIndex } from "./plots.js";
import { bindModelNotes } from "./notes.js";

const $ = id => document.getElementById(id);
const plot = new ResponsePlot($("chart"));
const updateModelNotes = bindModelNotes($("model-notes"), $("notes"));
const charts = { quarter: "side", LR: "front", FR: "side", full: "dashboard" };
const caseFields = {
  bump: ["height", "period"], step: ["height"], sine: ["height", "frequency"],
  random: ["height", "seed", "random_interval"], cornering: ["ay", "ramp"], accelerate: ["ax", "ramp"],
  bump_cornering: ["height", "period", "ay", "ramp"], combined: ["height", "period", "ax", "ay", "ramp", "speed"],
};
const tableHeads = ["Signal", "Unit", "Steady state", "2% settling time (s)", "Overshoot (%)", "Min", "Max", "RMS", "Absolute peak"];
const numberInputs = new Map();
const families = new Map();
let schema, project, family = "quarter", busy = false, running = false;
let result, index = 0, animation, lastTick, playTime;
let worker, requestId = 0;
const requests = new Map();

function el(tag, text, className) {
  const node = document.createElement(tag);
  if (text != null) node.textContent = text;
  if (className) node.className = className;
  return node;
}

function status(text, error = false) {
  $("status").textContent = text;
  $("status").toggleAttribute("data-error", error);
}

function rpc(action, payload = {}, { data, onProgress } = {}) {
  if (!worker) {
    worker = new Worker(new URL("worker.js", import.meta.url), { type: "module" });
    worker.onmessage = ({ data: reply }) => {
      const request = requests.get(reply.id);
      if (!request) return;
      if ("progress" in reply) {
        request.onProgress?.(reply.progress);
      } else {
        requests.delete(reply.id);
        if (reply.error) request.reject(new Error(reply.error));
        else request.resolve(reply.value);
      }
    };
    worker.onerror = event => {
      event.preventDefault();
      stopWorker(new Error(event.message || "The solver could not start."));
    };
  }
  const id = ++requestId;
  return new Promise((resolve, reject) => {
    requests.set(id, { resolve, reject, onProgress });
    try {
      worker.postMessage({ id, action, payload, data });
    } catch (error) {
      requests.delete(id);
      reject(error);
    }
  });
}

function stopWorker(error) {
  worker?.terminate();
  worker = null;
  for (const request of requests.values()) request.reject(error);
  requests.clear();
}

function options(select, choices, current) {
  select.replaceChildren(...Object.entries(choices).map(([value, label]) => new Option(label, value)));
  if (Object.hasOwn(choices, current)) select.value = current;
}

function tabButtons(parent, active, choose) {
  parent.replaceChildren(...Object.entries(schema.families).map(([key, label]) => {
    const button = el("button", label);
    button.type = "button";
    button.setAttribute("aria-pressed", key === active);
    button.addEventListener("click", () => choose(key));
    return button;
  }));
}

function numberField(label, unit, value, readOnly = false) {
  const field = el("label");
  const heading = el("div", `${label} `);
  heading.append(el("span", unit));
  const input = el("input");
  input.type = "number";
  input.step = "any";
  input.value = value ?? "";
  input.readOnly = readOnly;
  field.append(heading, input);
  return [field, input];
}

function viewChoices() {
  if (family === "full") return { dashboard: "Full-car dashboard", iso: "Isometric view", side: "Side view", front: "Front view" };
  return family === "LR" ? { front: "Front view" } : { side: "Side view" };
}

function selectedFamily() { return families.get(family); }

function setControls() {
  for (const id of ["analysis-fields", "import-project", "save-project", "settings"]) {
    $(id).disabled = busy || !project;
  }
  for (const id of ["history", "group", "compare", "play", "prev", "next", "time", "play-speed", "reset-zoom"]) {
    $(id).disabled = !result;
  }
  $("view").disabled = !result || family !== "full";
  $("gain").disabled = !result || $("view").value === "dashboard";
  $("export-format").disabled = $("export").disabled = !result || busy;
  $("restore").disabled = !result || busy;
  $("stop").hidden = !running;
  $("progress").hidden = !running;
}

function renderAnalysis() {
  tabButtons($("families"), family, selectFamily);
  const opt = project.analysis[family];
  options($("model"), schema.models[family], opt.model);
  const available = schema.family_scenarios[family][opt.model];
  options($("scenario"), Object.fromEntries(available.map(key => [key, schema.scenarios[key]])), opt.scenario);
  updateModelNotes(family, opt.model, opt.scenario);
  options($("target"), schema.targets[family], opt.target);
  const source = schema.sources[family];
  $("source-row").hidden = !source;
  if (source) options($("source"), source[1], opt[source[0]]);
  const acceleration = ["cornering", "accelerate"].includes(opt.scenario);
  $("target").disabled = acceleration;
  const hasDelay = ["FR", "full"].includes(family) && !acceleration;
  $("delay-row").hidden = !hasDelay;
  $("delay").checked = opt.rear_delay;
  const enabled = new Set(["duration", "dt", "start", ...(caseFields[opt.scenario] ?? ["speed", "seed"])]);
  if (hasDelay) enabled.add("speed");
  for (const [key, input] of numberInputs) {
    input.disabled = !enabled.has(key);
    input.value = opt[key] ?? "";
  }
  options($("view"), viewChoices(), selectedFamily().view);
  $("gain").value = selectedFamily().gain;
}

function selectFamily(nextFamily) {
  family = nextFamily;
  renderAnalysis();
  showResult(selectedFamily().selected);
}

function installProject(candidate) {
  project = candidate;
  for (const key of Object.keys(schema.families)) {
    families.set(key, { records: [], selected: null, next: 1, view: charts[key], gain: "1", group: null, signals: new Set() });
  }
  selectFamily(family);
}

function record(value) {
  const { family: resultFamily, model, project: snapshot } = value.meta;
  const target = families.get(resultFamily);
  const scenario = schema.scenarios[snapshot.analysis[resultFamily].scenario];
  value.id = String(target.next++);
  value.label = `${value.id.padStart(2, "0")} · ${schema.models[resultFamily][model]} · ${scenario}`;
  target.records.push(value);
  if (target.records.length > 20) target.records.shift();
  target.selected = value;
  if (resultFamily === family) showResult(value);
}

function buildTable(rows) {
  const table = el("table");
  const header = el("thead");
  const titles = el("tr");
  titles.append(...tableHeads.map(title => el("th", title)));
  header.append(titles);
  const body = el("tbody");
  for (const row of rows) {
    const tr = el("tr");
    tr.append(...row.map(value => el("td", value)));
    body.append(tr);
  }
  table.append(header, body);
  return table;
}

function resultInfo(value) {
  const { family: resultFamily, model, info } = value.meta;
  return {
    ...info,
    model_label: schema.models[resultFamily][model],
    warnings: (info.warnings ?? []).map(warning => schema.warning_labels[warning] ?? warning),
  };
}

function reportHtml(value, markup) {
  const { project: snapshot, statistics } = value.meta;
  const info = resultInfo(value);
  const report = el("main");
  report.append(el("h1", "Suspension Analysis Report"), el("p", `${snapshot.name} · ${info.model_label} · ${info.engine}`),
    el("p", `${value.time.length.toLocaleString("en")} samples · ${value.time[0]}–${value.time.at(-1)} s`));
  if (info.warnings.length) {
    const warnings = el("ul");
    warnings.append(...info.warnings.map(warning => el("li", warning)));
    report.append(warnings);
  }
  const chart = el("figure");
  chart.innerHTML = markup; // The plot serializes its own SVG with escaped labels.
  report.append(chart, el("h2", "Signal statistics"), el("p", schema.response_note), buildTable(statistics),
    el("h2", "Calculation details"), el("pre", JSON.stringify(info, null, 2)),
    el("h2", "Input snapshot"), el("pre", JSON.stringify(snapshot, null, 2)));
  return `<!doctype html><html lang="en"><meta charset="utf-8"><title>Suspension Analysis Report</title>
<style>body{font:15px system-ui,sans-serif;margin:36px;color:#172334}table{border-collapse:collapse;width:100%}
td,th{border:1px solid #ccd4df;padding:6px;text-align:left}pre{white-space:pre-wrap;background:#f1f5f9;padding:16px}
figure{margin:1em 0}svg{max-width:100%;height:auto}@media print{body{margin:12px}}</style>${report.outerHTML}</html>`;
}

function showResult(value) {
  pause();
  result = value;
  selectedFamily().selected = value;
  options($("history"), Object.fromEntries(selectedFamily().records.map(item => [item.id, item.label])), value?.id);
  const groups = Object.keys(value?.meta.groups ?? {});
  options($("group"), Object.fromEntries(groups.map(name => [name, schema.group_labels[name] ?? name])), selectedFamily().group);
  const info = value ? resultInfo(value) : null;
  $("result-note").textContent = value
    ? [value.label, `${value.time.length.toLocaleString("en")} samples`, ...info.warnings].join(" · ")
    : "Run a model to play the animation, view plots and export results.";
  $("snapshot").textContent = value ? JSON.stringify({ info, project: value.meta.project }, null, 2) : "";
  $("stats").replaceChildren(...(value ? [buildTable(value.meta.statistics)] : []));
  $("time").max = value ? value.time.length - 1 : 0;
  setControls();
  renderSignalOptions();
  setupDashboard();
  seek(0);
}

function renderSignalOptions() {
  const target = selectedFamily();
  target.group = $("group").value;
  const names = result?.meta.groups[target.group] ?? [];
  const visible = names.filter(name => target.signals.has(name));
  target.signals = new Set(visible.length ? visible : names.slice(0, 4));
  $("signals").replaceChildren(...names.map((name, i) => {
    const label = el("label", null, "check");
    const check = el("input");
    check.type = "checkbox";
    check.checked = target.signals.has(name);
    check.addEventListener("change", () => {
      if (check.checked) target.signals.add(name);
      else target.signals.delete(name);
      drawChart();
    });
    const text = el("span", name);
    text.style.color = curveColor(i);
    label.append(check, text);
    return label;
  }));
  drawChart();
}

function drawChart() {
  if (!result) { plot.set([], ""); return; }
  const target = selectedFamily();
  const names = result.meta.groups[target.group] ?? [];
  const byModel = new Map();
  if ($("compare").checked) {
    for (const item of target.records) {
      if (item.meta.comparison === result.meta.comparison) byModel.set(item.meta.model, item);
    }
  }
  const plotted = byModel.size ? [...byModel.values()] : [result];
  const dash = ["", "6 3", "2 3", "8 2 2 2", "12 4"];
  const series = plotted.flatMap((item, modelIndex) => names.flatMap((name, signalIndex) => {
    if (!target.signals.has(name) || !item.signals[name]) return [];
    return [{ name: plotted.length > 1 ? `${name} · ${item.meta.model}` : name,
      color: curveColor(signalIndex), dash: dash[modelIndex], time: item.time, values: item.signals[name] }];
  }));
  const units = new Set(names.filter(name => target.signals.has(name)).map(name => result.meta.units[name]));
  plot.set(series, [...units].join(" / "));
  const note = plotted.length > 1 ? `Overlay · ${plotted.map(item => item.meta.model).join(" / ")} · ${series.length} curves` : "";
  $("signals").querySelector(".comparison-note")?.remove();
  if (note) $("signals").append(el("span", note, "comparison-note"));
}

let meters = [], readout;
function setupDashboard() {
  const panel = $("dashboard");
  panel.replaceChildren();
  meters = [];
  readout = null;
  if (!result || family !== "full") return;
  const corners = ["FL", "FR", "RL", "RR"];
  const channels = ["heave_f", "heave_r", "roll_f", "roll_r", "roll_c", "warp", ...corners.map(corner => `corner_${corner}`)];
  const loads = corners.map(corner => [corner, `Normal load ${corner}`]);
  const forces = channels.map(name => [name, `Channel ${name}`]);
  for (const [title, definitions] of [["Normal wheel loads (N)", loads], ["Modal and direct forces (N)", forces]]) {
    const section = el("section");
    const list = el("div", null, "meters");
    let maximum = 1;
    for (const [, signal] of definitions) {
      for (const value of result.signals[signal] ?? []) maximum = Math.max(maximum, Math.abs(value));
    }
    for (const [label, signal] of definitions) {
      const row = el("div", null, "meter");
      const track = el("div", null, "meter-track");
      const bar = el("i");
      track.append(bar);
      const value = el("output", "—");
      row.append(el("span", label), track, value);
      list.append(row);
      if (result.signals[signal]) meters.push({ bar, value, data: result.signals[signal], maximum });
    }
    section.append(el("h4", title), list);
    panel.append(section);
  }
  readout = el("p");
  panel.append(readout);
}

function drawFrame() {
  const view = $("view").value;
  const dashboard = Boolean(result && view === "dashboard");
  $("dashboard").hidden = !dashboard;
  $("motion").toggleAttribute("hidden", dashboard);
  $("motion-title").textContent = dashboard ? "Full-car dashboard" : "Model animation";
  if (!dashboard) {
    drawMotion($("motion"), result, index, view, Number($("gain").value));
    return;
  }
  for (const { bar, value, data, maximum } of meters) {
    const sample = data[index];
    const width = 50 * Math.abs(sample) / maximum;
    bar.style.width = `${width}%`;
    bar.style.left = `${sample < 0 ? 50 - width : 50}%`;
    bar.toggleAttribute("data-negative", sample < 0);
    value.textContent = sample.toFixed(1);
  }
  const angles = { Roll: "Roll", Pitch: "Pitch", Twist: "Twist", "Roll front": "Front roll", "Roll rear": "Rear roll" };
  readout.textContent = Object.entries(angles).filter(([name]) => result.signals[name])
    .map(([name, label]) => `${label} ${result.signals[name][index].toFixed(3)}°`).join(" · ");
}

function seek(value) {
  index = result ? Math.max(0, Math.min(value, result.time.length - 1)) : 0;
  $("time").value = index;
  const time = result?.time[index] ?? 0;
  $("clock").textContent = `${time.toFixed(3)} / ${(result?.time.at(-1) ?? 0).toFixed(3)} s`;
  plot.setTime(time);
  drawFrame();
}

function pause() {
  if (animation !== undefined) cancelAnimationFrame(animation);
  animation = undefined;
  $("play").textContent = "Play";
}

function tick(now) {
  playTime += (now - lastTick) * Number($("play-speed").value) / 1000;
  lastTick = now;
  const next = sampleIndex(result.time, playTime);
  if (next !== index) seek(next);
  if (next === result.time.length - 1) pause();
  else animation = requestAnimationFrame(tick);
}

async function run(all = false) {
  if (busy || !project) return;
  const kind = family;
  const snapshot = structuredClone(project);
  const opt = snapshot.analysis[kind];
  const candidates = all ? Object.keys(schema.models[kind]) : [opt.model];
  const models = candidates.filter(model => schema.family_scenarios[kind][model].includes(opt.scenario));
  busy = running = true;
  $("progress").value = 0;
  setControls();
  let completed = 0;
  try {
    for (const model of models) {
      opt.model = model;
      status(`Running ${schema.models[kind][model]} (${completed + 1} of ${models.length})…`);
      record(await rpc("run", { project: snapshot, family: kind }, {
        onProgress: fraction => { $("progress").value = (completed + fraction) / models.length; },
      }));
      completed++;
    }
    status(`Completed ${completed} ${completed === 1 ? "model" : "models"}.`);
  } catch (error) {
    if (error.name === "AbortError") status(`Calculation stopped. Results saved in this session: ${completed}.`);
    else status(error.message, true);
  } finally {
    busy = running = false;
    setControls();
  }
}

async function whileBusy(operation) {
  if (busy) return;
  busy = true;
  setControls();
  try { await operation(); }
  catch (error) { status(error.message, true); }
  finally { busy = false; setControls(); }
}

let draft, editFamily, parameterInputs, editVersion = 0;
function saveDraft() {
  draft.name = $("edit-name").value.trim();
  for (const [key, input] of parameterInputs) {
    if (!input.readOnly) draft.parameters[editFamily][key] = input.value === "" ? null : Number(input.value);
  }
  return draft;
}

function renderParameters(effective) {
  tabButtons($("parameter-tabs"), editFamily, async next => {
    saveDraft();
    editFamily = next;
    renderParameters();
    await previewParameters();
  });
  parameterInputs = new Map();
  const nodes = [];
  if (editFamily === "quarter") {
    const automatic = el("input");
    automatic.type = "checkbox";
    automatic.checked = Boolean(draft.auto_damping?.quarter);
    automatic.addEventListener("change", async () => {
      saveDraft();
      draft.auto_damping = automatic.checked ? { quarter: true } : {};
      renderParameters();
      await previewParameters();
    });
    const label = el("label", null, "check");
    label.append(automatic, "Estimate damping from ratio ζ");
    nodes.push(label);
  }
  const grid = el("div", null, "numeric-grid");
  const local = schema.local[editFamily] ?? Object.keys(schema.fields[editFamily]);
  const base = draft.parameters[editFamily];
  for (const [key, [label, unit]] of Object.entries(schema.fields[editFamily])) {
    if (!local.includes(key) && !(key in (effective ?? {}))) continue;
    const automatic = editFamily === "quarter" && key === "cs" && draft.auto_damping?.quarter;
    const readOnly = !local.includes(key) || automatic;
    const [row, input] = numberField(label, unit, readOnly ? effective?.[key] ?? base[key] : base[key], readOnly);
    input.addEventListener("change", () => { if (!input.readOnly) previewParameters(); });
    parameterInputs.set(key, input);
    grid.append(row);
  }
  nodes.push(grid);
  $("parameters").replaceChildren(...nodes);
}

async function previewParameters() {
  const version = ++editVersion;
  try {
    const current = await rpc("parameters", { project: saveDraft() });
    if (version !== editVersion || !$("parameter-dialog").open) return;
    $("parameter-error").textContent = "";
    const effective = current.effective[editFamily];
    if (Object.keys(effective).some(key => Object.hasOwn(schema.fields[editFamily], key) && !parameterInputs.has(key))) {
      renderParameters(effective);
    } else {
      for (const [key, input] of parameterInputs) if (input.readOnly) input.value = effective[key] ?? "";
    }
    if (draft.auto_damping?.quarter) draft.parameters.quarter.cs = current.project.parameters.quarter.cs;
  } catch (error) {
    if (version === editVersion) $("parameter-error").textContent = error.message;
  }
}

let repairBytes, repairInputs = [];
function repairDialog(bytes, missing) {
  repairBytes = bytes;
  repairInputs = [];
  $("repairs").replaceChildren(...missing.map(field => {
    const label = el("label", `${field.label} ${field.unit}`);
    const input = el("input");
    input.placeholder = String(field.default);
    label.append(input, el("small", field.path.join(".")));
    repairInputs.push(input);
    return label;
  }));
  $("repair-error").textContent = "";
  $("repair-dialog").showModal();
}

function download(blob, filename) {
  const url = URL.createObjectURL(blob);
  const anchor = el("a");
  anchor.href = url;
  anchor.download = filename;
  anchor.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}

async function svgPng(markup) {
  const url = URL.createObjectURL(new Blob([markup], { type: "image/svg+xml;charset=utf-8" }));
  try {
    const image = new Image();
    image.src = url;
    await image.decode();
    const canvas = el("canvas");
    canvas.width = image.naturalWidth;
    canvas.height = image.naturalHeight;
    canvas.getContext("2d").drawImage(image, 0, 0);
    return await new Promise((resolve, reject) => canvas.toBlob(blob => {
      if (blob) resolve(blob);
      else reject(new Error("Could not create a PNG."));
    }));
  } finally { URL.revokeObjectURL(url); }
}

$("analysis").addEventListener("submit", event => { event.preventDefault(); run(); });
$("run-all").addEventListener("click", () => run(true));
$("stop").addEventListener("click", () => stopWorker(new DOMException("Calculation stopped.", "AbortError")));
document.addEventListener("keydown", event => {
  if (event.key === "F5" && !busy && project && !document.querySelector("dialog[open]")) {
    event.preventDefault();
    if ($("analysis").reportValidity()) run();
  }
});
for (const id of ["model", "scenario", "target"]) {
  $(id).addEventListener("change", () => {
    const opt = project.analysis[family];
    opt[id] = $(id).value;
    const scenarios = schema.family_scenarios[family][opt.model];
    if (!scenarios.includes(opt.scenario)) opt.scenario = scenarios[0];
    renderAnalysis();
    if (id === "model") {
      const matching = selectedFamily().records.findLast(item => item.meta.model === opt.model);
      if (matching) showResult(matching);
    }
  });
}
$("source").addEventListener("change", () => { project.analysis[family][schema.sources[family][0]] = $("source").value; });
$("delay").addEventListener("change", () => { project.analysis[family].rear_delay = $("delay").checked; });
$("history").addEventListener("change", () => showResult(selectedFamily().records.find(item => item.id === $("history").value)));
$("group").addEventListener("change", renderSignalOptions);
$("compare").addEventListener("change", drawChart);
$("reset-zoom").addEventListener("click", () => plot.reset());
$("view").addEventListener("change", () => { selectedFamily().view = $("view").value; setControls(); drawFrame(); });
$("gain").addEventListener("change", () => { selectedFamily().gain = $("gain").value; drawFrame(); });
$("prev").addEventListener("click", () => { pause(); seek(index - 1); });
$("next").addEventListener("click", () => { pause(); seek(index + 1); });
$("time").addEventListener("input", () => { pause(); seek(Number($("time").value)); });
$("play").addEventListener("click", () => {
  if (animation !== undefined) { pause(); return; }
  if (index === result.time.length - 1) seek(0);
  playTime = result.time[index];
  lastTick = performance.now();
  $("play").textContent = "Pause";
  animation = requestAnimationFrame(tick);
});

$("settings").addEventListener("click", () => {
  draft = structuredClone(project);
  editFamily = family;
  $("edit-name").value = draft.name;
  $("parameter-error").textContent = "";
  renderParameters();
  $("parameter-dialog").showModal();
  previewParameters();
});
$("close-settings").addEventListener("click", () => $("parameter-dialog").close());
$("parameter-form").addEventListener("submit", async event => {
  event.preventDefault();
  $("apply-settings").disabled = true;
  try {
    ++editVersion;
    const { project: updated } = await rpc("parameters", { project: saveDraft() });
    project = updated;
    $("parameter-dialog").close();
    renderAnalysis();
    status("Settings applied. Run the model again to update the results.");
  } catch (error) { $("parameter-error").textContent = error.message; }
  finally { $("apply-settings").disabled = false; }
});
$("restore").addEventListener("click", () => whileBusy(async () => {
  const kind = family;
  project = await rpc("restore", { project, family: kind, snapshot: result.meta.project });
  renderAnalysis();
  status("Inputs and model-specific settings applied.");
}));
$("save-project").addEventListener("click", () => whileBusy(async () => {
  const value = await rpc("save-project", { project });
  download(new Blob([value], { type: "application/json" }), `${project.name}.json`);
  status("Project JSON exported.");
}));

$("import-project").addEventListener("click", () => $("project-file").click());
$("project-file").addEventListener("change", () => whileBusy(async () => {
  const input = $("project-file");
  const file = input.files[0];
  input.value = "";
  if (!file) return;
  const data = new Uint8Array(await file.arrayBuffer());
  const value = await rpc("import-project", {}, { data });
  if (value.missing) repairDialog(data, value.missing);
  else { installProject(value.project); status(`Imported ${file.name}.`); }
}));
$("close-repair").addEventListener("click", () => $("repair-dialog").close());
$("repair-form").addEventListener("submit", async event => {
  event.preventDefault();
  $("apply-repair").disabled = true;
  try {
    const { project } = await rpc("import-project", { repairs: repairInputs.map(input => input.value) }, { data: repairBytes });
    $("repair-dialog").close();
    installProject(project);
    status("Missing parameters filled and project imported.");
  } catch (error) { $("repair-error").textContent = error.message; }
  finally { $("apply-repair").disabled = false; }
});
$("export").addEventListener("click", () => whileBusy(async () => {
  const format = $("export-format").value;
  const current = result;
  const markup = plot.exportSvg();
  let blob;
  if (format === "svg") blob = new Blob([markup], { type: "image/svg+xml;charset=utf-8" });
  else if (format === "png") blob = await svgPng(markup);
  else if (format === "html") blob = new Blob([reportHtml(current, markup)], { type: "text/html;charset=utf-8" });
  else {
    const value = await rpc("export-result", { format, meta: current.meta }, {
      data: { time: current.time, signals: current.signals },
    });
    const mime = { csv: "text/csv", npz: "application/octet-stream" };
    blob = new Blob([value], { type: mime[format] });
  }
  download(blob, `${current.meta.family}_${current.meta.model}.${format}`);
  status(`${format.toUpperCase()} exported.`);
}));

async function start() {
  $("retry").hidden = true;
  status("Loading the Python solver and NumPy…");
  drawMotion($("motion"), null, 0, "side", 1);
  try {
    schema = await rpc("bootstrap");
    $("response-note").textContent = schema.response_note;
    for (const [key, [label, unit]] of Object.entries(schema.inputs)) {
      const [field, input] = numberField(label, unit);
      input.addEventListener("input", () => { project.analysis[family][key] = input.value === "" ? null : Number(input.value); });
      numberInputs.set(key, input);
      $("numbers").append(field);
    }
    installProject(schema.project);
    status("Solver ready. Choose a model and input, then select Run model.");
  } catch (error) {
    status(`Could not load the solver. ${error.message}`, true);
    $("retry").hidden = false;
  }
}

$("retry").addEventListener("click", () => { stopWorker(new Error("Reloading the solver.")); start(); });
start();
