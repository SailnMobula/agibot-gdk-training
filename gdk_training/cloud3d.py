"""Shows point clouds in 3D in the browser (three.js, served from gdk_training/static).

    cloud3d.start({"front": lambda: front_points(), "back": lambda: back_points()})
    cloud3d.start({"map": lambda: cloud}, floor=lambda: cloud3d.Floor(image, geo), robot=lambda: pose)

Each layer is a function returning (N, 3) points x, y, z in m, or (N, 4) with a value per point in
the fourth column (for example reflectivity 0..255), or None when there is nothing to show. All
layers share one frame, z points up. A function may return the same object again; it is then not
sent again. Keep a large cloud or a floor in a variable and return that.

floor returns a Floor, a map image laid on z = 0. robot returns the pose of the robot as a 4x4
matrix in the same frame. labels returns (text, x, y, z) tuples. The page offers colour by layer,
by height and by value, a height band, the view from above and the point size.
Open http://localhost:8000 on the machine that runs the script.
"""

from __future__ import annotations

import io
import json
import os
import threading
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote

import cv2
import numpy as np

from gdk_training import show

STATIC = Path(__file__).with_name("static")
Points = Callable[[], np.ndarray | None]
COLORINGS = ("layer", "height", "value")  # the colour modes of the page, the first one is selected


@dataclass(frozen=True)
class Floor:
    image: np.ndarray  # BGR, row 0 at the top (maximum y), as from maps.grid_image
    origin_x: float  # m, map frame position of the bottom left corner of the image
    origin_y: float
    resolution: float  # m per pixel


@dataclass
class Slot:
    source: object = None  # the object the callback returned last time
    version: int = 0
    payload: bytes = b""
    meta: dict = field(default_factory=dict)
    error: str = ""


def pack(points: np.ndarray) -> tuple[bytes, dict]:
    """Turns (N, 3) or (N, 4) points into float32 x, y, z, value."""
    points = np.asarray(points, float).reshape(len(points), -1)
    out = np.zeros((len(points), 4), np.float32)
    out[:, :3] = points[:, :3]
    if points.shape[1] > 3:
        out[:, 3] = points[:, 3]
    return out.tobytes(), {"count": len(points), "values": points.shape[1] > 3}


class Scene:
    def __init__(self, layers: Mapping[str, Points], floor: Callable[[], Floor | None] | None,
                 robot: Callable[[], np.ndarray | None] | None, labels: Callable[[], list] | None, title: str,
                 colors: tuple[str, ...] = COLORINGS) -> None:
        self.layers = dict(layers)
        self.floor, self.robot, self.labels, self.title, self.colors = floor, robot, labels, title, list(colors)
        self.slots = {name: Slot() for name in self.layers}
        self.floor_slot = Slot()
        self.state: dict = {}
        self.lock = threading.Lock()

    def update(self) -> None:
        for name, function in self.layers.items():
            self.fill(self.slots[name], function, pack)
        if self.floor:
            self.fill(self.floor_slot, self.floor, self.encode_floor)
        robot, labels, errors = None, [], {}
        try:
            matrix = self.robot() if self.robot else None
            robot = None if matrix is None else [float(v) for v in np.asarray(matrix).ravel()]
        except Exception as error:  # A failing callback is shown in the page, the server keeps running.
            errors["robot"] = f"{type(error).__name__}: {error}"
        try:
            labels = [[str(t), float(x), float(y), float(z)] for t, x, y, z in (self.labels() if self.labels else [])]
        except Exception as error:
            errors["labels"] = f"{type(error).__name__}: {error}"
        floor = self.floor_slot
        with self.lock:
            self.state = {
                "title": self.title, "colors": self.colors,
                "layers": {name: {"version": slot.version, "count": 0, "values": False, **slot.meta}
                           for name, slot in self.slots.items()},
                "floor": None if floor.source is None else {"version": floor.version, **floor.meta},
                "robot": robot, "labels": labels,
                "errors": {**{name: slot.error for name, slot in self.slots.items() if slot.error},
                           **({"floor": floor.error} if floor.error else {}), **errors},
            }

    def fill(self, slot: Slot, function: Callable[[], object], encode: Callable[[object], tuple]) -> None:
        try:
            value = function()
            slot.error = ""
        except Exception as error:
            slot.error = f"{type(error).__name__}: {error}"
            return
        if value is slot.source:
            return
        payload, meta = (b"", {}) if value is None else encode(value)
        with self.lock:
            slot.source, slot.payload, slot.meta = value, payload, meta
            slot.version += 1

    @staticmethod
    def encode_floor(floor: Floor) -> tuple[bytes, dict]:
        height, width = floor.image.shape[:2]
        meta = {"width": width * floor.resolution, "height": height * floor.resolution,
                "x": floor.origin_x, "y": floor.origin_y}
        return cv2.imencode(".png", floor.image)[1].tobytes(), meta


def start(layers: Mapping[str, Points], floor: Callable[[], Floor | None] | None = None,
          robot: Callable[[], np.ndarray | None] | None = None, labels: Callable[[], list] | None = None,
          port: int = 8000, hz: float = 5.0, title: str = "",
          colors: tuple[str, ...] = COLORINGS) -> ThreadingHTTPServer | None:
    """Starts the server in the background and returns. With SHOW_ONCE=1 it prints the layer sizes instead.

    colors limits the colour modes of the page, e.g. ("height",). With one mode the choice is hidden.
    """
    if os.environ.get("SHOW_ONCE"):
        for name, function in layers.items():
            points = function()
            print(f"{name}: {'nothing' if points is None else f'{len(points)} points'}")
        if floor:
            print(f"floor: {'nothing' if floor() is None else floor().image.shape}")
        return None
    scene = Scene(layers, floor, robot, labels, title, colors)
    scene.update()

    def refresh() -> None:
        while True:
            time.sleep(1.0 / hz)
            scene.update()

    class Handler(BaseHTTPRequestHandler):
        def send(self, body: bytes, kind: str) -> None:
            self.send_response(200)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:
            path = self.path.split("?")[0]
            try:
                if path in ("/", "/index.html"):
                    self.send(PAGE.encode(), "text/html; charset=utf-8")
                elif path.startswith("/static/") and (STATIC / path[8:]).is_file() and "/" not in path[8:]:
                    self.send((STATIC / path[8:]).read_bytes(), "text/javascript")
                elif path == "/state.json":
                    with scene.lock:
                        self.send(json.dumps(scene.state).encode(), "application/json")
                elif path == "/floor.png":
                    self.send(scene.floor_slot.payload, "image/png")
                elif path.startswith("/layer/") and unquote(path[7:]) in scene.slots:
                    self.send(scene.slots[unquote(path[7:])].payload, "application/octet-stream")
                else:
                    self.send_error(404)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, *args: object) -> None:
            pass

    server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    threading.Thread(target=refresh, daemon=True).start()
    print(f"3D view on port {port}, open http://localhost:{port} in the browser on this machine")
    show.LINKS.append(f"http://localhost:{port} (3D view)")
    return server


def serve(layers: Mapping[str, Points], **options: object) -> None:
    """Starts the server and blocks until Ctrl+C."""
    if start(layers, **options) is None:
        return
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass


PAGE = r"""<!doctype html>
<html><head><meta charset="utf-8"><title>3D view</title>
<style>
  body { margin: 0; background: #111; color: #ddd; font: 13px system-ui, sans-serif; overflow: hidden; }
  #panel { position: absolute; top: 8px; left: 8px; background: rgba(0,0,0,.7); padding: 8px 10px;
           border-radius: 6px; line-height: 1.9; max-width: 420px; }
  #errors { color: #f88; white-space: pre-wrap; }
  input[type=number] { width: 4em; }
  .swatch { display: inline-block; width: 10px; height: 10px; margin-right: 3px; }
</style>
<script type="importmap">
{ "imports": { "three": "/static/three.module.min.js", "three/addons/OrbitControls.js": "/static/OrbitControls.js" } }
</script></head>
<body>
<div id="panel">
  <b id="title">3D view</b><br>
  <span id="layers"></span><br>
  <span id="colorbox">Colour <select id="color"><option value="layer">by layer</option>
  <option value="height">by height</option><option value="value">by value</option></select></span>
  Size <input id="size" type="number" value="0.03" step="0.01" min="0.005"><br>
  <label><input id="band" type="checkbox"> Height band</label>
  <input id="zmin" type="number" value="0.1" step="0.1"> to <input id="zmax" type="number" value="1.8" step="0.1"> m<br>
  <button id="top">View from above</button> <button id="side">View from the side</button>
  <label id="floorbox" hidden><input id="floor" type="checkbox" checked> Floor</label>
  <label><input id="pause" type="checkbox"> Pause</label><br>
  <span id="stats"></span><div id="errors"></div>
  <small>Mouse: left rotate, right pan, wheel zoom</small>
</div>
<script type="module">
import * as THREE from "three";
import { OrbitControls } from "three/addons/OrbitControls.js";

const COLORS = [0xff8800, 0x3399ff, 0x44dd66, 0xdd44dd, 0xffdd33, 0x33dddd];
const $ = (id) => document.getElementById(id);
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(innerWidth, innerHeight);
renderer.setPixelRatio(devicePixelRatio);
document.body.appendChild(renderer.domElement);
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x111111);
THREE.Object3D.DEFAULT_UP.set(0, 0, 1);
const camera = new THREE.PerspectiveCamera(60, innerWidth / innerHeight, 0.05, 300);
camera.up.set(0, 0, 1);
camera.position.set(-6, -3, 5);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0, 0.5);
controls.update();
addEventListener("resize", () => {
  camera.aspect = innerWidth / innerHeight; camera.updateProjectionMatrix();
  renderer.setSize(innerWidth, innerHeight);
});
scene.add(new THREE.AxesHelper(1));

const robot = new THREE.Group();
const body = new THREE.Mesh(new THREE.BoxGeometry(0.6, 0.45, 1.5), new THREE.MeshBasicMaterial({ color: 0x33cc66, wireframe: true }));
body.position.set(-0.03, 0, 0.75);
robot.add(body);
robot.add(new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0.05), 0.8, 0x33cc66));
robot.matrixAutoUpdate = false;
robot.visible = false;
scene.add(robot);

let floor = null, floorVersion = -1, colorsSet = false;
const labels = new THREE.Group();
scene.add(labels);
let labelKey = "";
const layers = {};
let state = null, firstView = true;
const tmp = new THREE.Color();

function colorFor(name, index, z, value) {
  const mode = $("color").value;
  if (mode === "height") return tmp.setHSL(0.7 - 0.7 * Math.min(Math.max((z + 0.2) / 2.7, 0), 1), 1, 0.5);
  if (mode === "value" && layers[name].values) return tmp.setHSL(0.7 - 0.7 * Math.min(Math.max(value / 150, 0), 1), 1, 0.25 + 0.4 * Math.min(value / 255, 1));
  return tmp.set(COLORS[index % COLORS.length]);
}

function rebuild(name) {
  const layer = layers[name];
  if (!layer.data) return;
  const band = $("band").checked, z0 = +$("zmin").value, z1 = +$("zmax").value;
  const index = Object.keys(layers).indexOf(name);
  const d = layer.data, n = d.length / 4;
  const pos = new Float32Array(n * 3), col = new Float32Array(n * 3);
  let k = 0;
  for (let i = 0; i < n; i++) {
    const z = d[4 * i + 2];
    if (band && (z < z0 || z > z1)) continue;
    pos.set([d[4 * i], d[4 * i + 1], z], 3 * k);
    const c = colorFor(name, index, z, d[4 * i + 3]);
    col.set([c.r, c.g, c.b], 3 * k);
    k++;
  }
  layer.shown = k;
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.BufferAttribute(pos.subarray(0, 3 * k), 3));
  geometry.setAttribute("color", new THREE.BufferAttribute(col.subarray(0, 3 * k), 3));
  layer.points.geometry.dispose();
  layer.points.geometry = geometry;
}

function rebuildAll() { Object.keys(layers).forEach(rebuild); stats(); }

function stats() {
  $("stats").textContent = Object.entries(layers).map(([n, l]) => `${n} ${l.shown ?? 0}/${l.count ?? 0}`).join("   ");
}

function makeLabel(text) {
  const canvas = document.createElement("canvas");
  canvas.width = 256; canvas.height = 64;
  const ctx = canvas.getContext("2d");
  ctx.font = "bold 36px sans-serif"; ctx.fillStyle = "#ffcc33"; ctx.fillText(text, 4, 44);
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(canvas), depthTest: false }));
  sprite.scale.set(1.0, 0.25, 1);
  sprite.center.set(0, 0);
  return sprite;
}

async function poll() {
  if ($("pause").checked) return;
  try {
    state = await (await fetch("/state.json")).json();
  } catch { return; }
  $("title").textContent = state.title || "3D view";
  if (!colorsSet) {
    colorsSet = true;
    for (const option of [...$("color").options]) if (!state.colors.includes(option.value)) option.remove();
    $("color").value = state.colors[0];
    $("colorbox").hidden = state.colors.length < 2;
  }
  $("errors").textContent = Object.entries(state.errors).map(([n, e]) => `${n}: ${e}`).join("\n");
  for (const [name, info] of Object.entries(state.layers)) {
    if (!layers[name]) {
      const points = new THREE.Points(new THREE.BufferGeometry(), new THREE.PointsMaterial({ size: +$("size").value, vertexColors: true }));
      scene.add(points);
      layers[name] = { points, version: -1 };
      const index = Object.keys(layers).length - 1;
      const label = document.createElement("label");
      label.innerHTML = `<input type="checkbox" checked> <span class="swatch" style="background:#${COLORS[index % COLORS.length].toString(16).padStart(6, "0")}"></span>${name} `;
      label.querySelector("input").onchange = (e) => { points.visible = e.target.checked; };
      $("layers").appendChild(label);
    }
    const layer = layers[name];
    layer.count = info.count; layer.values = info.values;
    if (info.version !== layer.version) {
      layer.version = info.version;
      layer.data = new Float32Array(await (await fetch(`/layer/${encodeURIComponent(name)}?v=${info.version}`)).arrayBuffer());
      rebuild(name);
    }
  }
  $("floorbox").hidden = !state.floor;
  if (state.floor && state.floor.version !== floorVersion) {
    floorVersion = state.floor.version;
    const texture = await new THREE.TextureLoader().loadAsync(`/floor.png?v=${floorVersion}`);
    texture.magFilter = THREE.NearestFilter;
    texture.colorSpace = THREE.SRGBColorSpace;
    if (floor) { scene.remove(floor); floor.geometry.dispose(); }
    floor = new THREE.Mesh(new THREE.PlaneGeometry(state.floor.width, state.floor.height),
                           new THREE.MeshBasicMaterial({ map: texture, transparent: true, opacity: 0.85 }));
    floor.position.set(state.floor.x + state.floor.width / 2, state.floor.y + state.floor.height / 2, -0.01);
    floor.visible = $("floor").checked;
    scene.add(floor);
  }
  robot.visible = !!state.robot;
  if (state.robot) {
    robot.matrix.set(...state.robot);
    robot.matrixWorldNeedsUpdate = true;
    if (firstView) {
      firstView = false;
      const p = new THREE.Vector3().setFromMatrixPosition(robot.matrix);
      controls.target.set(p.x, p.y, 0.5);
      camera.position.set(p.x - 6, p.y - 3, 5);
    }
  }
  const key = JSON.stringify(state.labels);
  if (key !== labelKey) {
    labelKey = key;
    labels.clear();
    for (const [text, x, y, z] of state.labels) { const s = makeLabel(text); s.position.set(x, y, z); labels.add(s); }
  }
  stats();
}

$("color").onchange = rebuildAll;
$("band").onchange = rebuildAll;
$("zmin").onchange = rebuildAll;
$("zmax").onchange = rebuildAll;
$("size").onchange = () => Object.values(layers).forEach((l) => { l.points.material.size = +$("size").value; });
$("floor").onchange = (e) => { if (floor) floor.visible = e.target.checked; };
$("top").onclick = () => { const t = controls.target; camera.position.set(t.x, t.y - 0.01, t.z + 15); controls.update(); };
$("side").onclick = () => { const t = controls.target; camera.position.set(t.x, t.y - 10, t.z + 1); controls.update(); };

let busy = false;
setInterval(async () => { if (busy) return; busy = true; try { await poll(); } finally { busy = false; } }, 200);
renderer.setAnimationLoop(() => { controls.update(); renderer.render(scene, camera); });
</script></body></html>
"""
