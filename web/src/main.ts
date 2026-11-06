import {
  AbstractMesh,
  ArcRotateCamera,
  Color3,
  Color4,
  Engine,
  DirectionalLight,
  HemisphericLight,
  Mesh,
  MeshBuilder,
  Quaternion,
  Scene,
  SceneLoader,
  StandardMaterial,
  TransformNode,
  Vector3,
} from "@babylonjs/core";
import "@babylonjs/loaders/glTF";

const WS_URL = `ws://${location.hostname}:9771`;

interface WallSpec {
  pos: [number, number, number];
  half_extents: [number, number, number];
}
type ObstacleSpec =
  | { kind: "box"; pos: [number, number, number]; half_extents: [number, number, number] }
  | { kind: "cylinder"; pos: [number, number, number]; radius: number; half_height: number };
interface ConfigMsg {
  type: "config";
  field: {
    half_x: number;
    half_z: number;
    wall_height: number;
    wall_thick: number;
    walls?: WallSpec[];
    obstacles?: ObstacleSpec[];
    // optional GLB visual model served from web/public (issue #42); when
    // it loads successfully the primitive walls/obstacles are skipped
    glb_url?: string | null;
  };
  robot: { half_extents: number };
  // per-robot visual/mechanism refs (issue #43); absent on pre-#43 servers
  robots?: RobotCfg[];
}
interface ModelSpecMsg {
  url: string;
  scale?: number;
  offset?: [number, number, number];
  rotation_deg?: [number, number, number];
}
interface IntakeSpecMsg {
  x_min: number;
  x_max: number;
  z_min: number;
  z_max: number;
  y_min: number;
  y_max: number;
}
interface RobotCfg {
  id: number;
  half_extents?: [number, number, number];
  model?: ModelSpecMsg | null;
  intake?: IntakeSpecMsg | null;
}
interface StateMsg {
  type: "state";
  tick: number;
  robots: {
    id: number;
    position: [number, number, number];
    rotation: [number, number, number, number];
    linvel: [number, number, number];
    contact_impulse: number;
  }[];
  pieces?: [number, number, number][];
  scores?: [number, number];
  held?: number[];
  match_state?: string;
  match_time?: number;
}
type Msg = ConfigMsg | StateMsg;

const hud = document.getElementById("hud")!;
const telem = (() => {
  const d = document.createElement("div");
  d.style.cssText = "position:fixed;right:8px;top:120px;background:rgba(10,14,20,.8);color:#cfd8e3;padding:10px 12px;border-radius:8px;font:11px monospace;z-index:10;min-width:190px";
  document.body.appendChild(d);
  return d;
})();
const canvas = document.getElementById("renderCanvas") as HTMLCanvasElement;
const engine = new Engine(canvas, true);
const scene = new Scene(engine);
scene.clearColor = new Color4(0.06, 0.07, 0.09, 1);

const camera = new ArcRotateCamera("cam", -Math.PI / 2, 0.55, 30, Vector3.Zero(), scene);
camera.attachControl(canvas, true);
camera.minZ = 0.05;
camera.maxZ = 300;
camera.lowerRadiusLimit = 5;
camera.upperRadiusLimit = 80;
{
  const hemi = new HemisphericLight("light", new Vector3(0.3, 1, 0.2), scene);
  hemi.intensity = 0.9;
  const dir = new DirectionalLight("dir", new Vector3(-0.4, -1, 0.3), scene);
  dir.intensity = 0.6;
  dir.position = new Vector3(4, 12, -3);
}

const ROBOT_COLORS = [Color3.Red(), new Color3(0.2, 0.4, 1), new Color3(0, 0.8, 0.4), Color3.Yellow()];

const robots = new Map<number, TransformNode>();
const robotCfgs = new Map<number, RobotCfg>();
const modelTried = new Set<number>();
const pieces: AbstractMesh[] = [];
const MAX_PIECES = 200;

// all 504 pieces are instances of one source mesh — a single draw call
// instead of per-mesh geometry/material (viz perf)
let pieceSrc: Mesh | null = null;
function pieceMesh(i: number): AbstractMesh {
  if (!pieceSrc) {
    pieceSrc = MeshBuilder.CreateSphere("piece-src", { diameter: 0.15, segments: 6 }, scene);
    const mat = new StandardMaterial("pm", scene);
    mat.diffuseColor = new Color3(1.0, 0.55, 0.1);
    pieceSrc.material = mat;
    pieceSrc.setEnabled(false); // template only — instances do the drawing
    pieces.push(pieceSrc);
  }
  while (pieces.length <= i) {
    const m = pieceSrc.createInstance(`piece-${pieces.length}`);
    pieces.push(m);
  }
  return pieces[i];
}

async function tryLoadFieldGlb(url: string): Promise<boolean> {
  try {
    const res = await SceneLoader.ImportMeshAsync("", "", url, scene);
    for (const m of res.meshes) {
      if (/fuel/i.test(m.name)) m.setEnabled(false);
    }
    console.log(`field GLB loaded: ${res.meshes.length} meshes`);
    document.title = "GLB field";
    return true;
  } catch (e) {
    console.warn(`field GLB ${url} failed, using primitives:`, e);
    document.title = "GLB FAILED " + e;
    return false;
  }
}
let robotHalf = 0.4;
let fieldCfg: ConfigMsg["field"] | null = null;
let built = false;

async function buildField() {
  if (!fieldCfg || built) return;
  built = true;
  const glbUrl = fieldCfg.glb_url;
  if (glbUrl) {
    // primitives are the fallback if the GLB 404s/fails to parse
    tryLoadFieldGlb(glbUrl).then((ok) => {
      if (!ok) buildPrimitives();
    });
    return;
  }
  buildPrimitives();
}

async function buildPrimitives() {
  if (!fieldCfg) return;
  document.title = "PRIMITIVE fallback";
  const { half_x, half_z, wall_height, wall_thick } = fieldCfg;

  const ground = MeshBuilder.CreateGround("ground", { width: half_x * 2, height: half_z * 2 }, scene);
  const gm = new StandardMaterial("gm", scene);
  gm.diffuseColor = new Color3(0.15, 0.4, 0.15);
  ground.material = gm;

  // Prefer the real 2026 REBUILT field model (issue #42). The GLB shares
  // our world frame (meters, y-up, x = field length), so no transform is
  // needed. Staged "Fuel" decor meshes are hidden — simulated pieces come
  // from the state stream. Fall back to primitives when the asset was not
  // fetched (`npm run fetch-assets`, see web/ASSETS.md).
  if (fieldCfg.glb_url) {
    try {
      const result = await SceneLoader.ImportMeshAsync("", location.origin, fieldCfg.glb_url, scene);
      for (const mesh of result.meshes) {
        if (/Fuel/i.test(mesh.name)) mesh.setEnabled(false);
      }
      ground.visibility = 0; // GLB brings its own carpet
      hud.textContent = "connected (GLB field), waiting for state…";
      return;
    } catch (err) {
      console.warn("field GLB unavailable, using primitives:", err);
    }
  }

  const wm = new StandardMaterial("wm", scene);
  wm.diffuseColor = new Color3(0.6, 0.6, 0.65);
  if (fieldCfg.walls?.length) {
    // real wall segments from the field spec
    for (const w of fieldCfg.walls) {
      const [hx, hy, hz] = w.half_extents;
      const wall = MeshBuilder.CreateBox("wall", { width: hx * 2, height: hy * 2, depth: hz * 2 }, scene);
      wall.position = new Vector3(w.pos[0], w.pos[1], w.pos[2]);
      wall.material = wm;
    }
  } else {
    const walls: [number, number, number, number][] = [
      [0, -half_z, half_x * 2, wall_thick],
      [0, half_z, half_x * 2, wall_thick],
      [-half_x, 0, wall_thick, half_z * 2],
      [half_x, 0, wall_thick, half_z * 2],
    ];
    for (const [x, z, w, d] of walls) {
      const wall = MeshBuilder.CreateBox("wall", { width: w, height: wall_height, depth: d }, scene);
      wall.position = new Vector3(x, wall_height / 2, z);
      wall.material = wm;
    }
  }

  const om = new StandardMaterial("om", scene);
  om.diffuseColor = new Color3(0.75, 0.5, 0.15);
  for (const o of fieldCfg.obstacles ?? []) {
    let mesh: Mesh;
    if (o.kind === "cylinder") {
      mesh = MeshBuilder.CreateCylinder("obstacle", { diameter: o.radius * 2, height: o.half_height * 2 }, scene);
    } else {
      const [hx, hy, hz] = o.half_extents;
      mesh = MeshBuilder.CreateBox("obstacle", { width: hx * 2, height: hy * 2, depth: hz * 2 }, scene);
    }
    mesh.position = new Vector3(o.pos[0], o.pos[1], o.pos[2]);
    mesh.material = om;
  }
}

function robotMesh(id: number): TransformNode {
  let m = robots.get(id);
  if (!m) {
    // root node receives the physics pose; children are display-only
    const root = new TransformNode(`robot-${id}`, scene);
    m = root;
    const cfg = robotCfgs.get(id);
    const [hx, hy, hz] = cfg?.half_extents ?? [robotHalf, robotHalf, robotHalf];

    // primitive fallback (pre-#43 path, also kept until/if the GLB loads)
    const box = MeshBuilder.CreateBox(`robot-${id}-box`, { width: hx * 2, height: hy * 2, depth: hz * 2 }, scene);
    box.parent = root;
    box.metadata = { fallback: true };
    const mat = new StandardMaterial(`rm-${id}`, scene);
    mat.diffuseColor = ROBOT_COLORS[id % ROBOT_COLORS.length];
    box.material = mat;
    // heading marker on top so yaw is visible
    const nose = MeshBuilder.CreateBox("nose", { width: hx, height: 0.1, depth: 0.12 }, scene);
    nose.parent = root;
    nose.position = new Vector3(hx * 0.6, hy + 0.05, 0);
    nose.metadata = { fallback: true };
    const nm = new StandardMaterial("nm", scene);
    nm.diffuseColor = Color3.White();
    nose.material = nm;

    // intake mechanism proxy (issue #43 v1): translucent chassis-relative
    // box over the declared capture region
    if (cfg?.intake) {
      const i = cfg.intake;
      const intake = MeshBuilder.CreateBox(
        `robot-${id}-intake`,
        { width: i.x_max - i.x_min, height: i.y_max - i.y_min, depth: i.z_max - i.z_min },
        scene,
      );
      intake.parent = root;
      intake.position = new Vector3(
        (i.x_min + i.x_max) / 2,
        (i.y_min + i.y_max) / 2,
        (i.z_min + i.z_max) / 2,
      );
      const im = new StandardMaterial(`im-${id}`, scene);
      im.diffuseColor = new Color3(0.1, 0.9, 0.5);
      im.emissiveColor = new Color3(0.05, 0.3, 0.15);
      im.alpha = 0.35;
      intake.material = im;
    }
    robots.set(id, m);
  }
  return m;
}

// Load the archetype's GLB once (issue #43). On success the fallback
// box/nose are disabled; on failure (asset not fetched, bad URL) the
// primitives stay. The model spec's scale/offset/rotation orient the
// asset into the chassis frame (+x forward, +y up).
async function loadRobotModel(id: number, root: TransformNode, cfg: RobotCfg) {
  if (!cfg.model || modelTried.has(id)) return;
  modelTried.add(id);
  try {
    const result = await SceneLoader.ImportMeshAsync("", cfg.model.url, "", scene);
    const holder = new TransformNode(`robot-${id}-model`, scene);
    holder.parent = root;
    const deg = Math.PI / 180;
    const urlRot = new URLSearchParams(location.search).get("rot");
    const [rx, ry, rz] = urlRot
      ? urlRot.split(",").map(Number) as [number, number, number]
      : (cfg.model.rotation_deg ?? [0, 0, 0]);
    // euler applied X then Y then Z (quaternion product qz*qy*qx)
    holder.rotationQuaternion = Quaternion.RotationAxis(new Vector3(0, 0, 1), rz * deg)
      .multiply(Quaternion.RotationAxis(new Vector3(0, 1, 0), ry * deg))
      .multiply(Quaternion.RotationAxis(new Vector3(1, 0, 0), rx * deg));
    const [ox, oy, oz] = cfg.model.offset ?? [0, 0, 0];
    holder.position = new Vector3(ox, oy, oz);
    const s = cfg.model.scale ?? 1;
    holder.scaling = new Vector3(s, s, s);
    for (const mesh of result.meshes) {
      if (!mesh.parent) mesh.parent = holder;
    }
    for (const child of root.getChildMeshes()) {
      if (child.metadata?.fallback) child.setEnabled(false);
    }
  } catch (err) {
    console.warn(`robot ${id} GLB unavailable, using box:`, err);
  }
}

function connect() {
  const ws = new WebSocket(WS_URL);
  ws.onopen = () => (hud.textContent = "connected, waiting for state…");
  ws.onclose = () => {
    hud.textContent = "disconnected — retrying in 1s";
    setTimeout(connect, 1000);
  };
  ws.onerror = () => ws.close();
  ws.onmessage = (ev) => {
    const msg: Msg = JSON.parse(ev.data);
    if (msg.type === "config") {
      fieldCfg = msg.field;
      robotHalf = msg.robot.half_extents;
      for (const rc of msg.robots ?? []) robotCfgs.set(rc.id, rc);
      buildField();
      return;
    }
    // game pieces (free ones only; held/scored are not in the list)
    if ((msg as any).hub_active !== undefined) {
      const h = (msg as any).hub_active;
      document.title = h === null ? "REBUILT sim" : `${h === 0 ? "RED" : "BLUE"} HUB ACTIVE`;
      const banner = document.getElementById("hub-banner") || (() => {
        const d = document.createElement("div");
        d.id = "hub-banner";
        d.style.cssText = "position:fixed;top:8px;left:50%;transform:translateX(-50%);padding:8px 18px;border-radius:8px;font:700 18px monospace;z-index:20;display:none";
        document.body.appendChild(d);
        return d;
      })();
      if (h === null) {
        banner.style.display = "none";
      } else {
        banner.style.display = "block";
        banner.textContent = `${h === 0 ? "RED" : "BLUE"} HUB ACTIVE`;
        banner.style.background = h === 0 ? "rgba(200,30,30,.85)" : "rgba(30,60,200,.85)";
        banner.style.color = "#fff";
      }
    }
    // telemetry rail (issue: match-ops UI): per-robot live stats
    {
      const rows: string[] = [`<b>tick ${(msg as any).tick ?? "?"} ${(msg as any).match_state ?? ""} ${((msg as any).match_time ?? 0).toFixed(0)}s</b>`,
        `score ${(msg as any).scores?.join("–") ?? "—"}`];
      robots.forEach((node, id) => {
        const p = node.position;
        const held = ((msg as any).held ?? [])[id] ?? 0;
        const mech = ((msg as any).mech_angles ?? [])[id];
        rows.push(`r${id} (${p.x.toFixed(1)},${p.z.toFixed(1)}) held=${held}${mech != null ? ` mech=${mech.toFixed(2)}` : ""}`);
      });
      telem.innerHTML = rows.join("<br>");
    }
    const plist = msg.pieces ?? [];
    for (let i = 0; i < MAX_PIECES; i++) {
      const m = pieceMesh(i);
      if (i < plist.length) {
        m.setEnabled(true);
        m.position.set(plist[i][0], plist[i][1] + 0.15, plist[i][2]);
      } else {
        m.setEnabled(false);
      }
    }

    let lines = [`tick ${msg.tick}`];
    if (msg.match_state) {
      lines.push(
        `${msg.match_state} ${msg.match_time?.toFixed(1) ?? "?"}s` +
          (msg.scores ? `  score ${msg.scores[0]}-${msg.scores[1]}` : ""),
      );
    }
    for (const r of msg.robots) {
      const m = robotMesh(r.id);
      const rc = robotCfgs.get(r.id);
      if (rc?.model) loadRobotModel(r.id, m, rc);
      m.position.set(r.position[0], r.position[1], r.position[2]);
      m.rotationQuaternion = new Quaternion(
        r.rotation[0], r.rotation[1], r.rotation[2], r.rotation[3],
      );
      const speed = Math.hypot(r.linvel[0], r.linvel[2]).toFixed(2);
      lines.push(
        `r${r.id} pos(${r.position[0].toFixed(2)}, ${r.position[2].toFixed(2)}) v=${speed} m/s` +
          (r.contact_impulse > 0 ? ` contact=${r.contact_impulse}` : ""),
      );
    }
    hud.textContent = lines.join("\n");
  };
}

connect();
engine.runRenderLoop(() => scene.render());
window.addEventListener("resize", () => engine.resize());
