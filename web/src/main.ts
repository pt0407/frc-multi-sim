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
  StandardMaterial,
  Vector3,
} from "@babylonjs/core";

const WS_URL = `ws://${location.hostname}:9771`;

interface ConfigMsg {
  type: "config";
  field: { half_x: number; half_z: number; wall_height: number; wall_thick: number };
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
let robotHalf = 0.4;
let fieldCfg: ConfigMsg["field"] | null = null;
let built = false;

function buildField() {
  if (!fieldCfg || built) return;
  built = true;
  const { half_x, half_z, wall_height, wall_thick } = fieldCfg;

  const ground = MeshBuilder.CreateGround("ground", { width: half_x * 2, height: half_z * 2 }, scene);
  const gm = new StandardMaterial("gm", scene);
  gm.diffuseColor = new Color3(0.15, 0.4, 0.15);
  ground.material = gm;

  const wm = new StandardMaterial("wm", scene);
  wm.diffuseColor = new Color3(0.6, 0.6, 0.65);
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
    let lines = [`tick ${msg.tick}`];
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
