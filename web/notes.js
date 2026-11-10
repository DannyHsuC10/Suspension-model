const sourceUrl = new URL("model-notes.md", import.meta.url);
const katexUrl = "https://cdn.jsdelivr.net/npm/katex@0.19.1/dist/";

let source;
let renderer;

async function readSections() {
  const response = await fetch(sourceUrl);
  if (!response.ok) throw new Error("Model notes could not be loaded.");
  const [, ...parts] = (await response.text()).split(/^<!-- section: ([\w.]+) -->\s*$/m);
  const sections = new Map();
  for (let i = 0; i < parts.length; i += 2) sections.set(parts[i], parts[i + 1].trim());
  return sections;
}

function loadStyles() {
  const link = document.createElement("link");
  link.rel = "stylesheet";
  link.href = `${katexUrl}katex.min.css`;
  link.integrity = "sha384-WwGtaC6ywCHxA5cstpj/3+xIZvTez8dq2IcBQaXRkenHZ7FVVq1ZROtURwfbtTlG";
  link.crossOrigin = "anonymous";
  return new Promise((resolve, reject) => {
    link.onload = resolve;
    link.onerror = () => { link.remove(); reject(new Error("Equation styles could not be loaded.")); };
    document.head.append(link);
  });
}

async function loadRenderer() {
  const [{ default: markdownIt }, { default: katex }] = await Promise.all([
    import("https://cdn.jsdelivr.net/npm/markdown-it@14.1.0/+esm"),
    import(`${katexUrl}katex.mjs`),
    loadStyles(),
  ]);
  const markdown = markdownIt({ html: false });
  const renderMath = (text, displayMode) => katex.renderToString(text, { displayMode, throwOnError: false });
  markdown.inline.ruler.after("escape", "inline_math", (state, silent) => {
    if (state.src[state.pos] !== "$") return false;
    const match = /^\$([^$\n]+)\$/.exec(state.src.slice(state.pos));
    if (!match) return false;
    if (!silent) state.push("inline_math", "", 0).content = match[1];
    state.pos += match[0].length;
    return true;
  });
  markdown.renderer.rules.inline_math = (tokens, index) => renderMath(tokens[index].content, false);
  const renderFence = markdown.renderer.rules.fence;
  markdown.renderer.rules.fence = (tokens, index, ...args) => {
    const token = tokens[index];
    if (token.info.trim() !== "math") return renderFence(tokens, index, ...args);
    return `<div class="math-block">${renderMath(token.content, true)}</div>\n`;
  };
  return markdown;
}

const roadInputs = new Set(["step", "bump", "sine", "random", "iso_a", "iso_b", "iso_c", "bump_cornering", "combined"]);
const bumpInputs = new Set(["bump", "bump_cornering", "combined"]);

function selectedMarkdown(sections, family, model, scenario) {
  const read = key => {
    const section = sections.get(key);
    if (!section) throw new Error(`Missing model notes section: ${key}`);
    return section;
  };
  const parts = [read("overview"), "### 2. Spring–damper model", read(`model.${family}.${model}`)];
  if (family !== "quarter") parts.push(read(`model.${family}.rates`));
  if (!["single", "one_eighth", "2mass_2dof", "half_2dof"].includes(model)) parts.push(read("model.tires"));

  parts.push("### 3. Input source function");
  if (roadInputs.has(scenario)) {
    parts.push(read("input.road"));
    if (family === "FR" || family === "full") parts.push(read("input.rear_delay"));
    const road = bumpInputs.has(scenario) ? "bump" : scenario;
    parts.push(read(`input.${road}`));
    if (road.startsWith("iso_")) parts.push(read("input.iso_profile"));
  }
  if (["cornering", "accelerate", "bump_cornering", "combined"].includes(scenario)) {
    parts.push(read("input.acceleration"), read(`input.${scenario}`));
  }
  return parts.join("\n\n");
}

export function bindModelNotes(disclosure, article) {
  let selection;
  let revision = 0;
  let lastRendered;

  async function update() {
    if (!disclosure.open || !selection) return;
    const current = selection;
    const key = current.join("/");
    const request = ++revision;
    if (key === lastRendered) return;
    article.textContent = "Loading model notes…";
    lastRendered = undefined;
    let text;
    try {
      source ??= readSections().catch(error => { source = undefined; throw error; });
      text = selectedMarkdown(await source, ...current);
      renderer ??= loadRenderer();
      const markdown = await renderer;
      if (request !== revision) return;
      // The Markdown is shipped with the app; raw HTML is disabled.
      article.innerHTML = markdown.render(text);
      lastRendered = key;
    } catch {
      if (request !== revision) return;
      const message = document.createElement("p");
      message.textContent = text ? "Equation rendering is unavailable. Showing the Markdown source." : "Model notes could not be loaded.";
      article.replaceChildren(message);
      if (text) {
        const plain = document.createElement("pre");
        plain.textContent = text;
        article.append(plain);
      }
    }
  }

  disclosure.addEventListener("toggle", update);
  return (family, model, scenario) => {
    selection = [family, model, scenario];
    update();
  };
}
