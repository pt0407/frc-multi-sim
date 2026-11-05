import {
  ArcRotateCamera,
  Color3,
  Color4,
  Engine,
  HemisphericLight,
  Mesh,
  MeshBuilder,
  Quaternion,
  Scene,
  SceneLoader,
  StandardMaterial,
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
    glb_url?: string | null;
  };
  robot: { half_extents: number };
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
const canvas = document.getElementById("renderCanvas") as HTMLCanvasElement;
const engine = new Engine(canvas, true);
const scene = new Scene(engine);
scene.clearColor = new Color4(0.06, 0.07, 0.09, 1);

const camera = new ArcRotateCamera("cam", -Math.PI / 2, 0.9, 22, Vector3.Zero(), scene);
camera.attachControl(canvas, true);
new HemisphericLight("light", new Vector3(0.3, 1, 0.2), scene);

const ROBOT_COLORS = [Color3.Red(), new Color3(0.2, 0.4, 1), new Color3(0, 0.8, 0.4), Color3.Yellow()];

const robots = new Map<number, Mesh>();
const pieces: Mesh[] = [];
let robotHalf = 0.4;
let fieldCfg: ConfigMsg["field"] | null = null;
let built = false;

const MAX_PIECES = 200;

function pieceMesh(i: number): Mesh {
  while (pieces.length <= i) {
    const m = MeshBuilder.CreateSphere(`piece-${pieces.length}`, { diameter: 0.3, segments: 8 }, scene);
    const mat = new StandardMaterial(`pm-${pieces.length}`, scene);
    mat.diffuseColor = new Color3(1.0, 0.55, 0.1);
    m.material = mat;
    pieces.push(m);
  }
  return pieces[i];
}

async function tryLoadFieldGlb(url: string): Promise<boolean> {
  try {
    const res = await SceneLoader.ImportMeshAsync("", location.origin, url, scene);
    // staged Fuel props in the field model are sim-owned — hide them
    for (const m of res.meshes) {
      if (/fuel/i.test(m.name)) m.setEnabled(false);
    }
    console.log(`field GLB loaded: ${res.meshes.length} meshes`);
    return true;
  } catch (e) {
    console.warn(`field GLB ${url} failed, using primitives:`, e);
    return false;
  }
}

function buildField() {
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

function buildPrimitives() {
  if (!fieldCfg) return;
  const { half_x, half_z, wall_height, wall_thick } = fieldCfg;

  const ground = MeshBuilder.CreateGround("ground", { width: half_x * 2, height: half_z * 2 }, scene);
  const gm = new StandardMaterial("gm", scene);
  gm.diffuseColor = new Color3(0.15, 0.4, 0.15);
  ground.material = gm;

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

function robotMesh(id: number): Mesh {
  let m = robots.get(id);
  if (!m) {
    m = MeshBuilder.CreateBox(`robot-${id}`, { size: robotHalf * 2 }, scene);
    const mat = new StandardMaterial(`rm-${id}`, scene);
    mat.diffuseColor = ROBOT_COLORS[id % ROBOT_COLORS.length];
    m.material = mat;
    // heading marker on top so yaw is visible
    const nose = MeshBuilder.CreateBox("nose", { width: robotHalf, height: 0.1, depth: 0.12 }, scene);
    nose.parent = m;
    nose.position = new Vector3(robotHalf * 0.6, robotHalf + 0.05, 0);
    const nm = new StandardMaterial("nm", scene);
    nm.diffuseColor = Color3.White();
    nose.material = nm;
    robots.set(id, m);
  }
  return m;
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
      buildField();
      return;
    }
    // game pieces (free ones only; held/scored are not in the list)
    const plist = msg.pieces ?? [];
    for (let i = 0; i < MAX_PIECES; i++) {
      const m = pieces[i];
      if (!m) break;
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
