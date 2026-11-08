import {
  AbstractMesh,
  ArcRotateCamera,
  Color3,
  Color4,
  Engine,
  DirectionalLight,
  HemisphericLight,
  ImageProcessingConfiguration,
  Matrix,
  Material,
  Mesh,
  MeshBuilder,
  Node as BNode,
  Quaternion,
  Scene,
  SceneLoader,
  StandardMaterial,
  TransformNode,
  Vector3,
  Axis,
  Space,
} from "@babylonjs/core";
import "@babylonjs/loaders/glTF";
import { DracoCompression } from "@babylonjs/core/Meshes/Compression/dracoCompression";

// robot CAD ships Draco-compressed; serve the decoder locally instead of
// relying on the Babylon CDN (offline-friendly, no version drift)
DracoCompression.Configuration.decoder.wasmUrl = "/draco/draco_wasm_wrapper_gltf.js";
DracoCompression.Configuration.decoder.wasmBinaryUrl = "/draco/draco_decoder_gltf.wasm";

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
interface MechSpecMsg {
  joint_type?: string;
  pivot: [number, number, number];
}
// [[joint]] kinematic mechanism spec — mirrors src/archetype.rs JointSpec.
// mode: spin | position | steer | wheel; driver: command | intake | shooter
interface JointCfgMsg {
  name: string;
  mv: string;
  scope?: string | null;
  mode?: string;
  driver?: string;
  axis?: [number, number, number] | null;
  pivot?: [number, number, number] | null;
  max_vel?: number;
  accel?: number;
  limits?: [number, number] | null;
  wheel_radius?: number;
}
interface JointStateMsg {
  name: string;
  mode: string;
  pos: number[];
  vel: number[];
}
interface RobotCfg {
  id: number;
  half_extents?: [number, number, number];
  model?: ModelSpecMsg | null;
  intake?: IntakeSpecMsg | null;
  mechs?: MechSpecMsg[];
  joints?: JointCfgMsg[];
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
    joints?: JointStateMsg[];
  }[];
  pieces?: [number, number, number][];
  scores?: [number, number];
  held?: number[];
  match_state?: string;
  match_time?: number;
}
type Msg = ConfigMsg | StateMsg;

const hud = document.getElementById("load-msg")!;
const loadEl = document.getElementById("load")!;
const telemEl = document.getElementById("telem")!;
function hideLoad() { loadEl.classList.add("done"); }
const telem = telemEl;
const canvas = document.getElementById("renderCanvas") as HTMLCanvasElement;
const engine = new Engine(canvas, false); // MSAA off — perf over edge smoothing
const scene = new Scene(engine);
(window as any).__scene = scene;
scene.clearColor = new Color4(0.06, 0.07, 0.09, 1);

const camera = new ArcRotateCamera("cam", -Math.PI / 2, 0.55, 30, Vector3.Zero(), scene);
camera.attachControl(canvas, true);
camera.minZ = 0.05;
camera.maxZ = 300;
camera.lowerRadiusLimit = 5;
camera.upperRadiusLimit = 80;
camera.lowerBetaLimit = 0.15;
camera.upperBetaLimit = 1.4;
camera.panningSensibility = 40;
// camera presets: 1=overview 2=follow-robot 3=red 4=blue (match-ops UX)
addEventListener("keydown", (e) => {
  if (e.key === "1") { camera.alpha = -Math.PI / 2; camera.beta = 0.55; camera.radius = 30; camera.target = Vector3.Zero(); }
  if (e.key === "2") { const r = robots.get(0); if (r) camera.target = r.position; }
  if (e.key === "3") { camera.alpha = -Math.PI / 2 + 0.4; camera.target = new Vector3(-3.5, 0, 0); }
  if (e.key === "4") { camera.alpha = -Math.PI / 2 - 0.4; camera.target = new Vector3(3.5, 0, 0); }
});
camera.wheelPrecision = 40;
{
  const hemi = new HemisphericLight("light", new Vector3(0.3, 1, 0.2), scene);
  hemi.intensity = 1.1;
  const dir = new DirectionalLight("dir", new Vector3(-0.4, -1, 0.3), scene);
  dir.intensity = 1.6;
  dir.position = new Vector3(4, 12, -3);
  // cool fill from the opposite side so shadowed faces don't go muddy
  const fill = new DirectionalLight("fill", new Vector3(0.5, -0.7, -0.4), scene);
  fill.intensity = 0.45;
  fill.position = new Vector3(-6, 8, 6);
  scene.ambientColor = new Color3(0.45, 0.45, 0.5);
  // filmic-ish tone mapping: rolls off highlights, deepens shadows
  const ipc = scene.imageProcessingConfiguration;
  ipc.toneMappingEnabled = true;
  ipc.toneMappingType = ImageProcessingConfiguration.TONEMAPPING_ACES;
  ipc.exposure = 1.15;
  ipc.contrast = 1.15;
}

const ROBOT_COLORS = [Color3.Red(), new Color3(0.2, 0.4, 1), new Color3(0, 0.8, 0.4), Color3.Yellow()];

const robots = new Map<number, TransformNode>();
const mechArms = new Map<number, Mesh>(); // robot id -> arm mesh (animated by mech_angles)
let telemTick = 0;
const intakeRollers = new Map<number, Mesh>(); // spinning intake roller
const flywheels = new Map<number, Mesh>(); // shooter flywheel disc
const shotFlash = new Map<number, { mesh: Mesh; ttl: number }>();
const prevHeld = new Map<number, number>();
let intakeOn: boolean[] = [];
const robotCfgs = new Map<number, RobotCfg>();
const modelTried = new Set<number>();
const modelOk = new Set<number>(); // CAD loaded — primitive proxies stay hidden
const pieces: AbstractMesh[] = [];
const MAX_PIECES = 600;
// shot tracers: emissive rings that ride the newest spawned pieces for
// ~1.5s so shot arcs are visible (live pieces grow in count only via
// shots — new indices = just-fired balls)
let prevPieceCount = 0;
const tracers: { mesh: Mesh; idx: number; ttl: number }[] = [];

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
    const live = res.meshes.filter((m) => {
      if (/fuel/i.test(m.name)) { m.setEnabled(false); return false; }
      return m.isEnabled() && !m.isAnInstance;
    });
    // the field never moves — merge ~3k primitives into one mesh PER
    // MATERIAL (~28 draw calls instead of ~3k; multiMultiMaterials would
    // keep a submesh per source mesh and defeat the purpose)
    let merged = 0, sourceCount = 0;
    const byMat = new Map<Material, Mesh[]>();
    for (const m of live) {
      if (!(m instanceof Mesh) || !m.material) continue;
      const arr = byMat.get(m.material) ?? [];
      arr.push(m);
      byMat.set(m.material, arr);
    }
    for (const group of byMat.values()) {
      if (group.length < 2) continue;
      // partition by sideOrientation — MergeMeshes rejects mixed winding
      const bySide = new Map<number, Mesh[]>();
      for (const m of group) {
        const arr = bySide.get(m.sideOrientation) ?? [];
        arr.push(m);
        bySide.set(m.sideOrientation, arr);
      }
      for (const g of bySide.values()) {
        if (g.length < 2) continue;
        if (Mesh.MergeMeshes(g, true, true, undefined, false, false)) {
          merged++;
          sourceCount += g.length;
        }
      }
    }
    console.log(`field GLB loaded: ${res.meshes.length} meshes -> ${merged} merged (${sourceCount} src)`);
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
    for (const m of result.meshes) m.freezeWorldMatrix();
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
    mesh.freezeWorldMatrix();
  }
  // static geometry never moves — freeze world matrices so Babylon
  // skips per-frame matrix recompute (500+ meshes get expensive)
  ground.freezeWorldMatrix();
  for (const m of scene.meshes) {
    if (m.name === "wall" || m.name === "obstacle" || m.name === "field") {
      m.freezeWorldMatrix();
    }
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
      intake.metadata = { fallback: true }; // hidden when GLB loads

      // spinning intake roller at the robot's front lip
      const roller = MeshBuilder.CreateCylinder(`robot-${id}-roller`, { diameter: 0.11, height: i.z_max - i.z_min }, scene);
      roller.parent = root;
      roller.rotation.x = Math.PI / 2; // axis along robot z
      roller.position = new Vector3(0.5, 0.08, 0);
      const rm = new StandardMaterial(`rm-${id}`, scene);
      rm.diffuseColor = new Color3(0.15, 0.15, 0.15);
      rm.emissiveColor = new Color3(0.02, 0.02, 0.02);
      roller.material = rm;
      roller.metadata = { fallback: true }; // proxy — hidden once CAD loads
      intakeRollers.set(id, roller);

      // shooter flywheel — spins whenever the bot is enabled, kick on shots
      const fw = MeshBuilder.CreateCylinder(`robot-${id}-fw`, { diameter: 0.18, height: 0.06 }, scene);
      fw.parent = root;
      fw.position = new Vector3(-0.2, 0.52, 0);
      const fm = new StandardMaterial(`fm-${id}`, scene);
      fm.diffuseColor = new Color3(0.9, 0.25, 0.1);
      fm.emissiveColor = new Color3(0.35, 0.08, 0.02);
      fw.material = fm;
      fw.metadata = { fallback: true };
      flywheels.set(id, fw);

      // muzzle flash marker shown briefly when a shot fires
      const fl = MeshBuilder.CreateBox(`robot-${id}-flash`, { size: 0.16 }, scene);
      fl.parent = root;
      fl.position = new Vector3(-0.2, 0.62, 0);
      const fmm = new StandardMaterial(`fmm-${id}`, scene);
      fmm.diffuseColor = new Color3(1.0, 0.9, 0.2);
      fmm.emissiveColor = new Color3(1.0, 0.7, 0.1);
      fl.material = fmm;
      fl.metadata = { fallback: true };
      fl.setEnabled(false);
      shotFlash.set(id, { mesh: fl, ttl: 0 });
    }
    robots.set(id, m);
  }
  return m;
}

const cadMechs = new Map<number, { pivot: TransformNode; axis: Vector3 }>();
// continuous spinners: shooter group (kind "shooter"), intake rollers
// (kind "intake"). angle accumulates in the render loop.
interface CadSpin { pivot: TransformNode; axis: Vector3; angle: number; kind: "shooter" | "intake"; }
const cadSpins = new Map<number, CadSpin[]>();
interface CadWheel { pivot: TransformNode; axis: Vector3; angle: number; radius: number; }
interface CadTurret {
  pivot: TransformNode;      // yaw pivot at module center
  upAxis: Vector3;           // holder-space up (0,0,1 for this Z-up export)
  yawSign: number;           // maps chassis yaw -> pivot angle
  fwd: Vector3;              // wheel rolling direction at bind, chassis frame
  wheel?: CadWheel;          // nested spin pivot (child of turret pivot)
}
const cadTurrets = new Map<number, CadTurret[]>();
// [[joint]] kinematic mechanisms: server-simulated state (pos/vel per
// channel) drives CAD pivots. channels[i] ↔ pos[i] for steer/wheel;
// spin/position joints have one state channel shared by all pivots.
interface CadJointChannel {
  pivot: TransformNode;
  axis: Vector3;      // rotation axis in pivot-parent space
  center: Vector3;    // holder-space bounds centre (wheel→turret reparent)
  pivotPos: Vector3;  // holder-space pivot position at bind
  home: number;       // steer: turret facing angle at bind (chassis frame)
  angle: number;      // anim-test accumulator (normal mode uses pos[])
}
interface CadJoint {
  spec: JointCfgMsg;
  channels: CadJointChannel[];
  upAxis: Vector3;    // steer pivots rotate about holder-space up
  yawSign: number;
}
const cadJoints = new Map<number, CadJoint[]>();
const jointStates = new Map<number, JointStateMsg[]>(); // latest per robot
const lastRobot = new Map<number, { vel: Vector3; quat: Quaternion }>();
let matchEnabled = false;
// animation test mode: ignores physics/match state and cycles every bound
// CAD mechanism so pivots/axes can be verified visually. ?anim=1 enables
// it at load; the home-screen button enables it live.
let animTest = new URLSearchParams(location.search).has("anim");
(window as any).frcAnimTest = () => { animTest = true; };
(window as any).frcScene = scene;

const stripOcc = (s: string) => s.replace(/^occurrence of /, "").replace(/\s*<\d+>$/, "");
const SHOOTER_SPIN = /^(Blades|Main Drum.*|.*Compliant Wheel.*|.*Thin Aluminum Roller.*|.*Hood Roller.*)$/;
const INTAKE_ROLLER = /^(1" Roller Deadaxle|2" Roller Assembly)$/;
const MK5_MODULE = /^SDS MK5n$/; // stripOcc removes the " <n>" occurrence suffix
const MK5_TURRET = /^SDS MK5 Turret Assembly$/;
const MK5_WHEEL = /^SDS MK5 Molded Wheel$/;

// holder-space AABB of a mesh set -> {center, extents}. The holder's world
// matrix is inverted so results live in model (GLB) coordinates.
// Mesh transform relative to the model holder, composed purely from each
// ancestor's LOCAL matrix (_localMatrix, refreshed by computeWorldDeep).
// Deliberately avoids _worldMatrix/getWorldMatrix: those caches proved
// unreliable for this GLB's deeply-nested instanced hierarchy and produced
// pivots ~10m off the robot.
function meshHolderMatrix(m: TransformNode, holder: TransformNode): Matrix | null {
  const locals: Matrix[] = [];
  for (let n: TransformNode | null = m; n && n !== holder; n = n.parent as TransformNode | null) {
    const lm = (n as unknown as { _localMatrix?: Matrix })._localMatrix;
    if (!lm) return null;
    locals.push(lm);
  }
  let acc = Matrix.Identity();
  for (const lm of locals) acc = lm.multiply(acc);
  return acc;
}

function holderBounds(meshes: AbstractMesh[], holder: TransformNode) {
  interface MB { c: Vector3; min: Vector3; max: Vector3 }
  const per: MB[] = [];
  const c = new Vector3();
  for (const m of meshes) {
    const rel = meshHolderMatrix(m, holder);
    if (!rel) continue;
    const bi = m.getRawBoundingInfo();
    const bmin = bi.minimum, bmax = bi.maximum;
    const mn = new Vector3(1e9, 1e9, 1e9), mx = new Vector3(-1e9, -1e9, -1e9);
    let cx = 0, cy = 0, cz = 0, n = 0;
    for (const px of [bmin.x, bmax.x])
      for (const py of [bmin.y, bmax.y])
        for (const pz of [bmin.z, bmax.z]) {
          Vector3.TransformCoordinatesToRef(new Vector3(px, py, pz), rel, c);
          mn.minimizeInPlace(c); mx.maximizeInPlace(c);
          cx += c.x; cy += c.y; cz += c.z; n++;
        }
    per.push({ c: new Vector3(cx / n, cy / n, cz / n), min: mn, max: mx });
  }
  if (!per.length) return { center: new Vector3(), extents: new Vector3() };
  const med = new Vector3(
    per.map((p) => p.c.x).sort((a, b) => a - b)[per.length >> 1],
    per.map((p) => p.c.y).sort((a, b) => a - b)[per.length >> 1],
    per.map((p) => p.c.z).sort((a, b) => a - b)[per.length >> 1],
  );
  const inl = per.filter((p) => Vector3.Distance(p.c, med) < 1.0);
  const use = inl.length ? inl : per;
  const min = new Vector3(1e9, 1e9, 1e9), max = new Vector3(-1e9, -1e9, -1e9);
  for (const p of use) { min.minimizeInPlace(p.min); max.maximizeInPlace(p.max); }
  return { center: min.add(max).scale(0.5), extents: max.subtract(min) };
}

// Pick the rotation axle of a roughly-cylindrical part: the axis whose two
// complementary bbox extents are most nearly equal (wheel: thickness is
// smallest; roller: length is largest — both leave dia≈dia).
function axleAxis(extents: Vector3): { axis: Vector3; radius: number } | null {
  const e = [extents.x, extents.y, extents.z];
  let best = -1, bestScore = Infinity, bestR = 0;
  for (let i = 0; i < 3; i++) {
    const j = (i + 1) % 3, k = (i + 2) % 3;
    const a = e[j], b = e[k];
    if (a < 0.004 || b < 0.004) continue;
    const score = Math.abs(a - b) / (a + b);
    if (score < bestScore) { bestScore = score; best = i; bestR = (a + b) / 4; }
  }
  if (best < 0 || bestScore > 0.4) return null;
  const axis = best === 0 ? Vector3.Right() : best === 1 ? Vector3.Up() : Vector3.Forward();
  return { axis, radius: bestR };
}

// Bind the real CAD mechanism subtrees to physics joints / sim state.
// Force fresh world matrices parent-before-child. computeWorldMatrix reads
// the parent's cached matrix, so array order alone isn't enough — sort by
// hierarchy depth (and include ancestors like the holder that aren't in the
// import result).
function computeWorldDeep(nodes: TransformNode[]) {
  const set = new Set<BNode>();
  for (const n of nodes) {
    for (let p: BNode | null = n; p; p = p.parent) set.add(p);
  }
  const depth = new Map<BNode, number>();
  const dOf = (n: BNode): number => {
    let d = depth.get(n);
    if (d === undefined) {
      d = n.parent ? dOf(n.parent as BNode) + 1 : 0;
      depth.set(n, d);
    }
    return d;
  };
  [...set]
    .filter((n) => typeof (n as BNode).computeWorldMatrix === "function")
    .sort((a, b) => dOf(a) - dOf(b))
    .forEach((n) => n.computeWorldMatrix(true));
}

function bindCadMechs(
  id: number,
  root: TransformNode,
  holder: TransformNode,
  result: { meshes: AbstractMesh[]; transformNodes: TransformNode[] },
) {
  const cfg = robotCfgs.get(id);
  const all: TransformNode[] = [...result.transformNodes, ...result.meshes];
  // attach top-level imported nodes to the holder so the whole CAD rides
  // the robot transform (nested nodes keep their internal parents)
  for (const n of all) {
    if (!n.parent) n.parent = holder;
  }
  // world matrices are lazy — without this, bounding boxes and setParent's
  // world-preservation read identity/stale matrices and scatter the model
  computeWorldDeep(all);
  const spins: CadSpin[] = [];
  const turrets: CadTurret[] = [];

  const inSubtree = (n: TransformNode, re: RegExp) => {
    for (let p: BNode | null = n; p; p = p.parent) {
      if (re.test(stripOcc(p.name ?? ""))) return true;
    }
    return false;
  };
  // innermost-wins: skip n when a descendant also matches (the "occurrence
  // of X" wrapper parents the real "X" node — the inner node owns the parts)
  const hasDescendantMatching = (n: TransformNode, re: RegExp) => {
    // transform nodes only — primitive meshes keep "<part>_primitiveN"
    // names that would false-positive the unanchored patterns
    for (const d of n.getChildTransformNodes()) {
      if (re.test(stripOcc(d.name ?? ""))) return true;
    }
    return false;
  };
  // getChildMeshes() recurses; the glTF "occurrence of X" wrapper nodes
  // only parent transform nodes, so direct-only misses every real mesh
  const meshesOf = (n: TransformNode): AbstractMesh[] => {
    const ms = n.getChildMeshes();
    // instanced meshes (common in Onshape exports for repeated parts —
    // kitbot wheels/pulleys) are InstancedMesh, not Mesh
    if (n instanceof AbstractMesh && !ms.includes(n)) ms.push(n);
    return ms;
  };

  // PASS 1 — measure everything before any reparenting (world matrices are
  // still pristine; setParent preserves world pos but dirties bounds).
  interface SpinSpec { name: string; kind: "shooter" | "intake"; meshes: AbstractMesh[]; center: Vector3; axis: Vector3; }
  interface TurretSpec {
    meshes: AbstractMesh[]; center: Vector3;
    wheel?: { meshes: AbstractMesh[]; center: Vector3; axis: Vector3; radius: number };
    fwd: Vector3; yawSign: number;
  }
  const spinSpecs: SpinSpec[] = [];
  const turretSpecs: TurretSpec[] = [];
  const claimed = new Set<AbstractMesh>(); // a mesh belongs to exactly one mechanism
  // [[joint]] specs replace the hardcoded spin/turret bindings when the
  // archetype declares them (data-driven mechanism binding)
  const useJoints = (cfg?.joints?.length ?? 0) > 0;
  interface JMatch {
    node: TransformNode;
    meshes: AbstractMesh[];
    center: Vector3;
    extents: Vector3;
    detAxis: Vector3 | null; // auto-detected axle (holder space), if any
  }
  interface JBind { spec: JointCfgMsg; idx: number; matches: JMatch[]; fallbackAxis: Vector3 | null }
  const jointMatches: JBind[] = [];

  // --- deployable intake arm: physics revolute joint ---------------------
  const mech = cfg?.mechs?.find((m) => (m.joint_type ?? "revolute") === "revolute");
  const intake = all.find((n) => /Pivoting Intake Assembly/i.test(n.name ?? ""));
  let intakeParts: AbstractMesh[] = [];
  if (mech && intake) {
    intakeParts = meshesOf(intake);
    for (const m of intakeParts) claimed.add(m);
  }

  // shooter group + intake rollers (innermost name match only, so the
  // "occurrence of X" wrapper and the inner "X" don't double-bind)
  const shooterRoot = all.find((n) => /^32 - R2 Shooter & Feeder/.test(stripOcc(n.name ?? "")));
  const collectSpins = (re: RegExp, scope: RegExp | null, kind: "shooter" | "intake") => {
    for (const n of all) {
      const base = stripOcc(n.name ?? "");
      if (!re.test(base)) continue;
      if (hasDescendantMatching(n, re)) continue; // inner node wins
      if (scope && !inSubtree(n, scope)) continue;
      const meshes = meshesOf(n).filter((m) => !claimed.has(m));
      if (!meshes.length) continue;
      const b = holderBounds(meshes, holder);
      const ax = axleAxis(b.extents);
      if (!ax) continue;
      for (const m of meshes) claimed.add(m);
      spinSpecs.push({ name: base.slice(0, 24), kind, meshes, center: b.center, axis: ax.axis });
    }
  };
  if (!useJoints) {
    if (shooterRoot) collectSpins(SHOOTER_SPIN, /^32 - R2 Shooter & Feeder/, "shooter");
    if (intake) {
      // rollers live inside the arm subtree but must not be claimed by the
      // arm pivot itself — they spin independently
      for (const m of intakeParts) claimed.delete(m);
      collectSpins(INTAKE_ROLLER, /Pivoting Intake Assembly/, "intake");
      for (const m of intakeParts) claimed.add(m);
    }
  }

  // --- swerve modules -----------------------------------------------------
  const isDesc = (node: TransformNode, anc: TransformNode) => {
    for (let p: BNode | null = node; p; p = p.parent) if (p === anc) return true;
    return false;
  };
  const MrInv = root.computeWorldMatrix(true).invert();
  const Mh = holder.computeWorldMatrix(true);
  const upHolderC = new Vector3(0, 0, 1).applyRotationQuaternion(holder.rotationQuaternion ?? Quaternion.Identity());
  const yawSign = upHolderC.y >= 0 ? 1 : -1;
  // "occurrence of X" wrappers and inner "X" nodes both match — pick the
  // innermost so the pivot wraps exactly the part's own meshes
  const innermost = (cands: TransformNode[]) =>
    cands.find((t) => cands.every((o) => o === t || isDesc(t, o)));
  if (!useJoints) {
  for (const n of all) {
    if (!MK5_MODULE.test(stripOcc(n.name ?? ""))) continue;
    const turretNode = innermost(
      all.filter((t) => MK5_TURRET.test(stripOcc(t.name ?? "")) && isDesc(t, n)),
    );
    if (!turretNode) continue;
    const tMeshes = meshesOf(turretNode).filter((m) => !claimed.has(m));
    if (!tMeshes.length) continue;
    const tb = holderBounds(tMeshes, holder);
    for (const m of tMeshes) claimed.add(m);
    const wheelNode = innermost(
      all.filter((w) => MK5_WHEEL.test(stripOcc(w.name ?? "")) && isDesc(w, n)),
    );
    let wheel: TurretSpec["wheel"];
    let fwd = new Vector3(1, 0, 0);
    if (wheelNode) {
      const wMeshes = meshesOf(wheelNode).filter((m) => !claimed.has(m));
      if (wMeshes.length) {
        const wb = holderBounds(wMeshes, holder);
        const ax = axleAxis(wb.extents);
        if (ax) {
          for (const m of wMeshes) claimed.add(m);
          wheel = { meshes: wMeshes, center: wb.center, axis: ax.axis, radius: ax.radius * (holder.scaling.x || 1) };
          const axleW = Vector3.TransformNormal(ax.axis, Mh);
          const axleC = Vector3.TransformNormal(axleW, MrInv);
          fwd = Vector3.Cross(new Vector3(0, 1, 0), axleC);
          if (fwd.lengthSquared() < 1e-6) fwd = new Vector3(1, 0, 0); else fwd.normalize();
        }
      }
    }
    turretSpecs.push({ meshes: tMeshes, center: tb.center, wheel, fwd, yawSign });
  }
  } else {
    // --- [[joint]] specs: generic mechanism matching ---------------------
    // wheel joints bind before steer so a wheel nested inside the turret
    // assembly claims its meshes first (the turret then gets the rest)
    const order = (cfg?.joints ?? [])
      .map((s, idx) => ({ s, idx }))
      .sort((a, b) =>
        (a.s.mode === "wheel" ? 0 : a.s.mode === "steer" ? 1 : 2) -
        (b.s.mode === "wheel" ? 0 : b.s.mode === "steer" ? 1 : 2));
    // meshes inside the deployable arm are "claimed" by the arm pivot but
    // remain available to joints whose subtree lives inside the arm
    // (intake rollers swing with the arm AND spin independently)
    const armMeshSet = new Set(intakeParts);
    for (const { s: jspec, idx: jidx } of order) {
      const re = new RegExp(jspec.mv);
      const scopeRe = jspec.scope ? new RegExp(jspec.scope) : null;
      const matches: JMatch[] = [];
      for (const n of all) {
        const base = stripOcc(n.name ?? "");
        // Babylon splits multi-primitive parts into "<part>_primitiveN"
        // child meshes — the parent node owns them all; skip the splits
        if (/_primitive\d+$/.test(base)) continue;
        if (!re.test(base)) continue;
        if (hasDescendantMatching(n, re)) continue; // innermost wins
        if (scopeRe && !inSubtree(n, scopeRe)) continue;
        const nodeInArm = !!(intake && isDesc(n, intake));
        const meshes = meshesOf(n).filter(
          (m) => !claimed.has(m) || (nodeInArm && armMeshSet.has(m)),
        );
        if (!meshes.length) continue;
        const b = holderBounds(meshes, holder);
        for (const m of meshes) claimed.add(m);
        // pre-detect the axle now (holder space); non-cylindrical parts
        // like flat blades fall back to the joint's first detected axis
        const detAxis = jspec.mode === "steer"
          ? null
          : (axleAxis(b.extents)?.axis ?? null);
        matches.push({ node: n, meshes, center: b.center, extents: b.extents, detAxis });
      }
      if (!matches.length) {
        console.warn(`robot ${id}: joint "${jspec.name}" matched no CAD nodes`);
        continue;
      }
      // steer/wheel: one channel per drivetrain module. Channel order must
      // match the server's module order (FL,FR,BL,BR = +x+z,+x-z,-x+z,-x-z)
      // → sort by robot-frame x desc, then z desc.
      if (jspec.mode === "steer" || jspec.mode === "wheel") {
        const hm = Matrix.Compose(
          holder.scaling,
          holder.rotationQuaternion ?? Quaternion.Identity(),
          holder.position,
        );
        matches.sort((a, b) => {
          const ra = Vector3.TransformCoordinates(a.center, hm);
          const rb = Vector3.TransformCoordinates(b.center, hm);
          return rb.x - ra.x || rb.z - ra.z;
        });
      }
      jointMatches.push({
        spec: jspec,
        idx: jidx,
        matches,
        fallbackAxis: matches.map((m) => m.detAxis).find((a) => a !== null) ?? null,
      });
    }
    jointMatches.sort((a, b) => a.idx - b.idx); // spec order for state lookup
  }

  // PASS 2 — create pivots and reparent (world transforms preserved).
  // holder TRS maps holder/model space <-> chassis (robot) frame; shared
  // by the mech pivot and [[joint]] spec.pivot overrides
  const hm = Matrix.Compose(
    holder.scaling,
    holder.rotationQuaternion ?? Quaternion.Identity(),
    holder.position,
  );
  const hmInv = hm.invert();
  let mechPivot: TransformNode | null = null;
  let mechPivotPos = new Vector3();
  if (mech && intake && intakeParts.length) {
    const inv = hmInv;
    const pivotLocal = Vector3.TransformCoordinates(
      new Vector3(mech.pivot[0], mech.pivot[1], mech.pivot[2]), inv);
    const pivot = new TransformNode(`cad-mech-${id}`, scene);
    pivot.parent = holder;
    pivot.position = pivotLocal;
    const axis = Vector3.TransformNormal(new Vector3(0, 0, 1), inv).normalize();
    for (const p of intakeParts) p.setParent(pivot);
    cadMechs.set(id, { pivot, axis });
    mechPivot = pivot;
    mechPivotPos = pivotLocal;
  }
  if (useJoints) {
    const bindings: CadJoint[] = [];
    for (const jm of jointMatches) {
      const mode = jm.spec.mode ?? "spin";
      const channels: CadJointChannel[] = [];
      for (const mt of jm.matches) {
        // resolve the rotation axis BEFORE reparenting so a bad match
        // doesn't leave orphaned pivots
        let axis: Vector3 | null = null;
        if (jm.spec.axis) {
          // spec axis is chassis-frame → holder space (direction only)
          axis = Vector3.TransformNormal(
            new Vector3(jm.spec.axis[0], jm.spec.axis[1], jm.spec.axis[2]), hmInv,
          ).normalize();
        } else if (mode === "steer") {
          axis = new Vector3(0, 0, 1); // holder-space up (same as legacy turret)
        } else {
          // detected axle, else the joint's first detected axis (flat
          // parts like blades share the flywheel's axle direction)
          axis = mt.detAxis ?? jm.fallbackAxis;
        }
        if (!axis) {
          console.warn(`robot ${id}: joint "${jm.spec.name}": no axis for "${mt.node.name}"`);
          continue;
        }
        const pivot = new TransformNode(`j${jm.idx}-${id}-${channels.length}`, scene);
        const inArm = !!(mechPivot && intake && isDesc(mt.node, intake));
        if (inArm) {
          // mechanism inside the deployable arm — rides the mech pivot
          pivot.parent = mechPivot;
          pivot.position = mt.center.subtract(mechPivotPos);
        } else {
          pivot.parent = holder;
          pivot.position = jm.spec.pivot
            ? Vector3.TransformCoordinates(
                new Vector3(jm.spec.pivot[0], jm.spec.pivot[1], jm.spec.pivot[2]), hmInv,
              )
            : mt.center;
        }
        for (const m of mt.meshes) m.setParent(pivot);
        channels.push({
          pivot,
          axis,
          center: mt.center,
          pivotPos: pivot.position.clone(),
          home: 0,
          angle: 0,
        });
      }
      if (channels.length) {
        bindings.push({ spec: jm.spec, channels, upAxis: new Vector3(0, 0, 1), yawSign });
      }
    }
    // wheel pivots ride inside their module's steer (turret) pivot, and
    // steer home = turret facing at bind derived from the wheel axle
    const steerB = bindings.find((b) => (b.spec.mode ?? "") === "steer");
    const wheelB = bindings.find((b) => (b.spec.mode ?? "") === "wheel");
    if (steerB && wheelB) {
      const n = Math.min(steerB.channels.length, wheelB.channels.length);
      for (let i = 0; i < n; i++) {
        const wp = wheelB.channels[i];
        const sp = steerB.channels[i];
        wp.pivot.parent = sp.pivot;
        wp.pivot.position = wp.center.subtract(sp.pivotPos);
        // wheel axle → rolling direction in chassis frame → home yaw
        const axW = Vector3.TransformNormal(wp.axis, Mh);
        const axR = Vector3.TransformNormal(axW, MrInv);
        let fwd = Vector3.Cross(new Vector3(0, 1, 0), axR);
        if (fwd.lengthSquared() < 1e-6) fwd = new Vector3(1, 0, 0);
        else fwd.normalize();
        steerB.channels[i].home = Math.atan2(fwd.z, fwd.x);
      }
    }
    if (bindings.length) cadJoints.set(id, bindings);
    console.log(
      `robot ${id}: bound ${bindings.length} joints ` +
        `(${bindings.map((b) => `${b.spec.name}:${b.channels.length}`).join(",")})`,
    );
  } else {
  for (const s of spinSpecs) {
    const pivot = new TransformNode(`spin-${id}-${s.name}`, scene);
    if (s.kind === "intake" && mechPivot) {
      // rollers ride the deployable arm — parent under the mech pivot so
      // they swing down with it; position is pivot-relative
      pivot.parent = mechPivot;
      pivot.position = s.center.subtract(mechPivotPos);
    } else {
      pivot.parent = holder;
      pivot.position = s.center;
    }
    for (const m of s.meshes) m.setParent(pivot);
    spins.push({ pivot, axis: s.axis, angle: 0, kind: s.kind });
  }
  for (const t of turretSpecs) {
    const turretPivot = new TransformNode(`turret-${id}`, scene);
    turretPivot.parent = holder;
    turretPivot.position = t.center;
    for (const m of t.meshes) m.setParent(turretPivot);
    let wheel: CadWheel | undefined;
    if (t.wheel) {
      const wp = new TransformNode(`wheel-${id}`, scene);
      wp.parent = turretPivot;
      wp.position = t.wheel.center.subtract(t.center);
      for (const m of t.wheel.meshes) m.setParent(wp);
      wheel = { pivot: wp, axis: t.wheel.axis, angle: 0, radius: t.wheel.radius };
    }
    turrets.push({ pivot: turretPivot, upAxis: new Vector3(0, 0, 1), yawSign: t.yawSign, fwd: t.fwd, wheel });
  }
  }
  computeWorldDeep(all);
  if (spins.length) cadSpins.set(id, spins);
  if (turrets.length) cadTurrets.set(id, turrets);
  console.log(`robot ${id}: bound CAD mechs (arm=${cadMechs.has(id)} spins=${spins.length} turrets=${turrets.length} joints=${cadJoints.get(id)?.length ?? 0})`);
  mergeRobotMeshes(id, root, holder);
}

// The Spectrum export explodes each CAD part into dozens of primitives
// (~25k meshes = unplayable draw calls). MergeMeshes bakes world-space
// verts with per-material submeshes; we then re-anchor each merged mesh
// to its mechanism pivot (or the robot root for static parts) via the
// inverse bind world matrix so articulation still works.
function mergeRobotMeshes(id: number, root: TransformNode, holder: TransformNode) {
  const t0 = performance.now();
  const pivots: TransformNode[] = [];
  const cm = cadMechs.get(id);
  if (cm) pivots.push(cm.pivot);
  for (const jb of cadJoints.get(id) ?? []) {
    for (const ch of jb.channels) pivots.push(ch.pivot);
  }
  for (const sp of cadSpins.get(id) ?? []) pivots.push(sp.pivot);
  for (const tm of cadTurrets.get(id) ?? []) {
    pivots.push(tm.pivot);
    if (tm.wheel) pivots.push(tm.wheel.pivot);
  }
  const pset = new Set<TransformNode>(pivots);
  // owner pivot per mesh: nearest pivot ancestor, null = static chassis part
  const groups = new Map<TransformNode | null, Mesh[]>();
  for (const m of holder.getChildMeshes()) {
    if (!(m instanceof Mesh) || !m.isEnabled() || m.isAnInstance) continue;
    let owner: TransformNode | null = null;
    for (let p = m.parent; p; p = p.parent) {
      if (pset.has(p as TransformNode)) { owner = p as TransformNode; break; }
      if (p === holder) break;
    }
    const arr = groups.get(owner) ?? [];
    arr.push(m);
    groups.set(owner, arr);
  }
  const bindWorld = new Map<TransformNode, Matrix>();
  for (const p of [...pivots, root]) {
    bindWorld.set(p, p.computeWorldMatrix(true).clone());
  }
  let mergedMeshes = 0, sourceCount = 0;
  for (const [owner, meshes] of groups) {
    // group by material — one merged mesh per material = one draw call
    // per material (multiMultiMaterials keeps per-source submeshes and
    // would leave thousands of draw calls)
    const byMat = new Map<Material, Mesh[]>();
    for (const m of meshes) {
      if (!m.material) continue;
      const arr = byMat.get(m.material) ?? [];
      arr.push(m);
      byMat.set(m.material, arr);
    }
    for (const group of byMat.values()) {
      // partition by sideOrientation — MergeMeshes rejects mixed winding
      const bySide = new Map<number, Mesh[]>();
      for (const m of group) {
        const arr = bySide.get(m.sideOrientation) ?? [];
        arr.push(m);
        bySide.set(m.sideOrientation, arr);
      }
      for (const g of bySide.values()) {
        if (g.length < 2) continue;
        const res = Mesh.MergeMeshes(g, true, true, undefined, false, false);
        if (!res) continue;
        mergedMeshes++;
        sourceCount += g.length;
        const inv = (owner ? bindWorld.get(owner)! : bindWorld.get(root)!).invert();
        res.parent = owner ?? root;
        if (!res.rotationQuaternion) res.rotationQuaternion = Quaternion.Identity();
        inv.decompose(res.scaling, res.rotationQuaternion, res.position);
      }
    }
  }
  console.log(
    `robot ${id}: merged ${sourceCount} CAD meshes -> ${mergedMeshes} draw meshes ` +
    `(${(performance.now() - t0).toFixed(0)}ms)`,
  );
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
    for (const m of result.meshes) {
      // staged fuel props baked into the CAD are sim-owned — hide them
      if (/fuel/i.test(m.name)) m.setEnabled(false);
    }
    const holder = new TransformNode(`robot-${id}-model`, scene);
    holder.parent = root;
    const deg = Math.PI / 180;
    const params = new URLSearchParams(location.search);
    const urlRot = params.get("rot");
    const [rx, ry, rz] = urlRot
      ? urlRot.split(",").map(Number) as [number, number, number]
      : (cfg.model.rotation_deg ?? [0, 0, 0]);
    // euler applied X then Y then Z (quaternion product qz*qy*qx)
    holder.rotationQuaternion = Quaternion.RotationAxis(new Vector3(0, 0, 1), rz * deg)
      .multiply(Quaternion.RotationAxis(new Vector3(0, 1, 0), ry * deg))
      .multiply(Quaternion.RotationAxis(new Vector3(1, 0, 0), rx * deg));
    const urlOff = params.get("off");
    const [ox, oy, oz] = urlOff
      ? urlOff.split(",").map(Number) as [number, number, number]
      : (cfg.model.offset ?? [0, 0, 0]);
    holder.position = new Vector3(ox, oy, oz);
    const s = cfg.model.scale ?? 1;
    holder.scaling = new Vector3(s, s, s);
    for (const mesh of result.meshes) {
      if (!mesh.parent) mesh.parent = holder;
    }
    for (const child of root.getChildMeshes()) {
      if (child.metadata?.fallback) child.setEnabled(false);
    }
    modelOk.add(id);
    try {
      bindCadMechs(id, root, holder, result);
    } catch (e) {
      console.error(`robot ${id} CAD mech binding failed:`, e);
    }
  } catch (err) {
    console.warn(`robot ${id} GLB unavailable, using box:`, err);
    // surface the failure — a silent fallback looks like "the model is
    // just blocks" with no way to tell why
    document.title = `MODEL FAIL r${id}: ${err}`;
    const el = document.getElementById("load-msg");
    if (el) el.textContent = `robot ${id} model failed: ${err}`;
  }
}

let vizWs: WebSocket | null = null;
// home-screen lineup changes rebuild the world server-side; the viz config
// (per-robot model/mech refs) is sent per connection, so force a reconnect
// to pick up the new lineup
(window as any).frcReconnect = () => vizWs?.close();

function connect() {
  const ws = new WebSocket(WS_URL);
  vizWs = ws;
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
    // animated mechanism arms — one bar per robot pivoting at the
    // chassis front edge; angle comes from mech_angles (issue #57/#61)
    intakeOn = (msg as any).intake_active ?? [];
    (msg as any).held?.forEach((h: number, id: number) => {
      const prev = prevHeld.get(id) ?? h;
      if (h < prev) {
        const f = shotFlash.get(id);
        // CAD robots keep the flash proxy hidden — ttl still drives the
        // shooter spin kick below
        if (f) { f.ttl = 10; if (!modelOk.has(id)) f.mesh.setEnabled(true); }
      }
      prevHeld.set(id, h);
    });
    (msg as any).mech_angles?.forEach((a: number | null, id: number) => {
      if (a == null) return;
      const cad = cadMechs.get(id);
      if (cad) {
        if (!animTest) cad.pivot.rotationQuaternion = Quaternion.RotationAxis(cad.axis, a);
        return;
      }
      let arm = mechArms.get(id);
      const root = robots.get(id);
      if (!arm && root && !modelOk.has(id)) {
        arm = MeshBuilder.CreateBox(`mech-${id}`, { width: 0.7, height: 0.05, depth: 0.08 }, scene);
        arm.parent = root;
        arm.position = new Vector3(0.35, 0.15, 0);
        const m = new StandardMaterial(`mechm-${id}`, scene);
        m.diffuseColor = new Color3(1.0, 0.8, 0.2);
        arm.material = m;
        mechArms.set(id, arm);
      }
      if (arm) {
        arm.rotation.z = a; // joint angle from physics
        arm.setEnabled(true);
      }
    });
    // telemetry rail (issue: match-ops UI): per-robot live stats,
    // throttled — rebuilding the HTML every 50 Hz tick is wasted work
    telemTick = (telemTick + 1) % 10;
    if (telemTick === 0) {
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
        m.position.set(plist[i][0], plist[i][1], plist[i][2]);
      } else {
        m.setEnabled(false);
      }
    }
    // tag newly spawned pieces with a tracer
    if (plist.length > prevPieceCount) {
      for (let i = prevPieceCount; i < plist.length; i++) {
        const tm = MeshBuilder.CreateSphere(`tracer-${i}`, { diameter: 0.24, segments: 6 }, scene);
        const tmat = new StandardMaterial(`tmat-${i}`, scene);
        tmat.diffuseColor = new Color3(1, 0.9, 0.3);
        tmat.emissiveColor = new Color3(1, 0.8, 0.2);
        tmat.alpha = 0.6;
        tm.material = tmat;
        tracers.push({ mesh: tm, idx: i, ttl: 75 });
      }
    }
    prevPieceCount = plist.length;
    // tracers ride their piece + fade
    for (let i = tracers.length - 1; i >= 0; i--) {
      const t = tracers[i];
      if (t.ttl <= 0 || t.idx >= plist.length) {
        t.mesh.dispose();
        tracers.splice(i, 1);
        continue;
      }
      t.mesh.position.set(plist[t.idx][0], plist[t.idx][1], plist[t.idx][2]);
      (t.mesh.material as StandardMaterial).alpha = 0.6 * (t.ttl / 75);
      t.ttl--;
    }

    let lines = [`tick ${msg.tick}`];
    if (msg.match_state) {
      matchEnabled = !["disabled", "complete"].includes(msg.match_state);
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
      jointStates.set(r.id, r.joints ?? []);
      lastRobot.set(r.id, {
        vel: new Vector3(r.linvel[0], r.linvel[1], r.linvel[2]),
        quat: new Quaternion(r.rotation[0], r.rotation[1], r.rotation[2], r.rotation[3]),
      });
      const speed = Math.hypot(r.linvel[0], r.linvel[2]).toFixed(2);
      lines.push(
        `r${r.id} pos(${r.position[0].toFixed(2)}, ${r.position[2].toFixed(2)}) v=${speed} m/s` +
          (r.contact_impulse > 0 ? ` contact=${r.contact_impulse}` : ""),
      );
    }
    telem.textContent = ""; telem.innerHTML = lines.join("<br>"); hideLoad();
  };
}

connect();
const SPIN = { roller: 0.25, flywheel: 0.4, kick: 0.8 };
engine.runRenderLoop(() => {
  const dt = Math.min(engine.getDeltaTime() / 1000, 0.1);
  if (animTest) {
    // step-through inspection: one mechanism group animates at a time so
    // each binding can be verified visually without everything moving at once
    const t = performance.now() / 1000;
    const PHASES = ["intake arm", "shooter", "intake rollers", "swerve"];
    const phase = Math.floor(t / 5) % PHASES.length;
    const hud = document.getElementById("anim-hud");
    if (hud) {
      hud.style.display = "block";
      hud.textContent = `ANIM TEST — all joints cycling  (arm=${cadMechs.size} spins=${[...cadSpins.values()].flat().length} turrets=${[...cadTurrets.values()].flat().length} joints=${[...cadJoints.values()].reduce((n, js) => n + js.reduce((m, j) => m + j.channels.length, 0), 0)})`;
    }
    const arm = phase === 0 ? 0.9 * Math.sin(t * 1.1) - 0.1 : 0;
    const armDown = arm < 0.15;
    for (const cm of cadMechs.values()) {
      cm.pivot.rotationQuaternion = Quaternion.RotationAxis(cm.axis, arm);
    }
    // [[joint]] mechanisms: every joint gets a distinct inspectable motion
    // (steer sweeps, wheels spin, shooter pulses, positions oscillate)
    for (const joints of cadJoints.values()) {
      for (const jb of joints) {
        const mode = jb.spec.mode ?? "spin";
        for (let c = 0; c < jb.channels.length; c++) {
          const ch = jb.channels[c];
          let ang = 0;
          if (mode === "steer") {
            ang = jb.yawSign * (0.9 * Math.sin(t * 0.9 + c * 1.7) - ch.home);
          } else if (mode === "wheel") {
            ch.angle += 25 * dt;
            ang = ch.angle;
          } else if (mode === "position") {
            ang = 0.7 * Math.sin(t * 1.2 + c);
          } else {
            const speed = jb.spec.driver === "shooter"
              ? 30 + 40 * Math.abs(Math.sin(t * 2.2))
              : jb.spec.driver === "intake" ? 18 : 12;
            ch.angle += speed * dt;
            ang = ch.angle;
          }
          ch.pivot.rotationQuaternion = Quaternion.RotationAxis(ch.axis, ang);
        }
      }
    }
    for (const spins of cadSpins.values()) {
      for (const s of spins) {
        const want = s.kind === "shooter" ? phase === 1 : phase === 2;
        const speed = want ? (s.kind === "shooter" ? 30 + 40 * Math.abs(Math.sin(t * 2.2)) : 18) : 0;
        s.angle += speed * dt;
        s.pivot.rotationQuaternion = Quaternion.RotationAxis(s.axis, s.angle);
      }
    }
    for (const mods of cadTurrets.values()) {
      for (const m of mods) {
        const yaw = phase === 3 ? 0.8 * Math.sin(t * 0.9) : 0;
        m.pivot.rotationQuaternion = Quaternion.RotationAxis(m.upAxis, yaw);
        if (m.wheel) {
          m.wheel.angle += (phase === 3 ? 25 : 0) * dt;
          m.wheel.pivot.rotationQuaternion = Quaternion.RotationAxis(m.wheel.axis, m.wheel.angle);
        }
      }
    }
  } else {
  const animHud = document.getElementById("anim-hud");
  if (animHud) animHud.style.display = "none";
  // CAD mechanism animation: spins accumulate angle, turrets/wheels track
  // the latest physics velocity each frame
  for (const [id, spins] of cadSpins) {
    const kicking = (shotFlash.get(id)?.ttl ?? 0) > 0;
    for (const s of spins) {
      const speed = s.kind === "shooter"
        ? (matchEnabled ? (kicking ? 90 : 26) : 0)
        : (intakeOn[id] ? 16 : 0);
      s.angle += speed * dt;
      s.pivot.rotationQuaternion = Quaternion.RotationAxis(s.axis, s.angle);
    }
  }
  for (const [id, mods] of cadTurrets) {
    const st = lastRobot.get(id);
    if (!st) continue;
    const qinv = st.quat.clone().invert();
    const v = st.vel.applyRotationQuaternion(qinv);
    const speed = Math.hypot(v.x, v.z);
    for (const t of mods) {
      if (speed > 0.12) {
        const yaw = Math.atan2(v.z, v.x) - Math.atan2(t.fwd.z, t.fwd.x);
        const a = ((yaw + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI;
        t.pivot.rotationQuaternion = Quaternion.RotationAxis(t.upAxis, t.yawSign * a);
      }
      if (t.wheel) {
        t.wheel.angle += (speed / Math.max(t.wheel.radius, 0.02)) * dt;
        t.wheel.pivot.rotationQuaternion = Quaternion.RotationAxis(t.wheel.axis, t.wheel.angle);
      }
    }
  }
  // [[joint]] mechanisms: server-simulated joint state → CAD pivots.
  // steer/wheel have one state channel per module (pos[c]); spin/position
  // share a single channel across all matched subtrees (pos[0]).
  for (const [id, joints] of cadJoints) {
    const states = jointStates.get(id);
    if (!states) continue;
    for (let j = 0; j < joints.length; j++) {
      const jb = joints[j];
      const st = states[j];
      if (!st) continue;
      const mode = jb.spec.mode ?? "spin";
      for (let c = 0; c < jb.channels.length; c++) {
        const ch = jb.channels[c];
        if (mode === "steer") {
          const s = st.pos[c] ?? 0;
          ch.pivot.rotationQuaternion = Quaternion.RotationAxis(
            jb.upAxis, jb.yawSign * (s - ch.home),
          );
        } else {
          const a = mode === "wheel" ? (st.pos[c] ?? 0) : (st.pos[0] ?? 0);
          ch.pivot.rotationQuaternion = Quaternion.RotationAxis(ch.axis, a);
        }
      }
    }
  }
  }
  for (const [id, roller] of intakeRollers) {
    if (intakeOn[id]) roller.rotate(Axis.Y, SPIN.roller, Space.LOCAL);
  }
  for (const [id, fw] of flywheels) {
    const kicking = (shotFlash.get(id)?.ttl ?? 0) > 0;
    fw.rotate(Axis.Y, kicking ? SPIN.kick : SPIN.flywheel, Space.LOCAL);
  }
  for (const f of shotFlash.values()) {
    if (f.ttl > 0 && --f.ttl === 0) f.mesh.setEnabled(false);
  }
  scene.render();
});
window.addEventListener("resize", () => engine.resize());
