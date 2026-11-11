const runtime = "https://cdn.jsdelivr.net/pyodide/v314.0.7/full/";

async function loadEngine() {
  const { loadPyodide } = await import(`${runtime}pyodide.mjs`);
  const pyodide = await loadPyodide({ indexURL: runtime });
  const archiveUrl = new URL("engine.zip", import.meta.url);
  archiveUrl.search = new URL(import.meta.url).search;
  const archive = fetch(archiveUrl).then(response => {
    if (!response.ok) throw new Error(`Could not load the solver package (${response.status}).`);
    return response.arrayBuffer();
  });
  const [, bytes] = await Promise.all([pyodide.loadPackage("numpy"), archive]);
  pyodide.unpackArchive(bytes, "zip");
  return pyodide.runPython("from web_bridge import handle\nhandle");
}

let engine;
self.onmessage = async ({ data: { id, action, payload, data } }) => {
  try {
    engine ??= loadEngine();
    const handle = await engine;
    const result = handle(action, JSON.stringify(payload), data ?? null,
      progress => self.postMessage({ id, progress }));
    try {
      const value = result.toJs({ dict_converter: Object.fromEntries, create_pyproxies: false });
      const transfer = value.time
        ? [value.time.buffer, ...Object.values(value.signals).map(values => values.buffer)]
        : ArrayBuffer.isView(value) ? [value.buffer] : [];
      self.postMessage({ id, value }, transfer);
    } finally {
      result.destroy();
    }
  } catch (error) {
    const message = error.message.trim().split("\n").at(-1);
    self.postMessage({ id, error: message.replace(/^\w+(Error|Exception): /, "") });
  }
};
