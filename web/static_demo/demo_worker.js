// Web Worker: runs the REAL Python (agent + mock flat_lab world + keyword planner) in Pyodide. No server involved.
// Messages in:  {id, op, args}  -> out: {id, result} | {id, error};   progress: {type: "progress", text} / {type: "ready"} / {type: "fatal", error}
importScripts("demo_config.js");                       // sets self.PYODIDE_BASE (the Pyodide runtime on the jsDelivr CDN, pinned version)
const post = (m) => self.postMessage(m);
let call = null;

async function boot() {
  post({ type: "progress", text: "Downloading the Python runtime" });
  importScripts(self.PYODIDE_BASE + "pyodide.js");
  const py = await loadPyodide({ indexURL: self.PYODIDE_BASE });
  post({ type: "progress", text: "Loading the robot's code" });
  await py.loadPackage("pyyaml");
  try { await py.loadPackage("sqlite3"); } catch (e) { /* no sqlite3: the demo runs without the timeline */ }
  const zip = await (await fetch("gurdy_demo.zip")).arrayBuffer();
  py.unpackArchive(zip, "zip", { extractDir: "/home/pyodide/app" });
  py.runPython(`import sys\nsys.path[:0] = ["/home/pyodide/app/pylib", "/home/pyodide/app", "/home/pyodide/app/gurdy_core"]`);
  post({ type: "progress", text: "Starting the simulated robot" });
  call = py.runPython("from web.static_demo.session import call\ncall");
  call("reset");
  post({ type: "ready" });
}

boot().catch((e) => post({ type: "fatal", error: String((e && e.message) || e) }));

self.onmessage = (ev) => {
  const { id, op, args } = ev.data;
  try {
    if (!call) throw new Error("the simulator is not ready yet");
    post({ id, result: JSON.parse(call(op, JSON.stringify(args || []))) });
  } catch (e) {
    post({ id, error: String((e && e.message) || e) });
  }
};
