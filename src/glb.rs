//! Minimal GLB (binary glTF) reader for static field meshes (issue #42).
//!
//! Parses the JSON + BIN chunks directly — no `gltf` crate dependency —
//! and emits world-space triangle meshes for nodes that pass the
//! fieldcore-derived name filter (excludes Fuel pieces, tape, decals, and
//! fastener hardware, which are visual-only or too dense to be useful
//! colliders).
use anyhow::{bail, Context, Result};
use serde::Deserialize;
use std::collections::HashMap;
use std::path::Path;

/// Name substrings that exclude a node from collision geometry — mirrors
/// fieldcore's `shouldUseOfficialFieldCollider` regex list.
const EXCLUDE: &[&str] = &[
    "fuel", "tape", "vinyl", "sticker", "decal", "screw", "bearing", "spacer",
    "washer", "nut", "bolt", "pem", "rivet", "tread", "wheel", "gas spring",
    "rev-21-2246",
];

/// Skip primitives with more vertices than this (fieldcore uses 80k per
/// mesh; we apply it per-primitive, which is stricter).
const MAX_VERTS: usize = 80_000;

/// Bounds cull: the GLB includes geometry outside the field envelope
/// (stairs, bleachers, staging). Anything fully outside this box is not a
/// collision candidate. Field is 16.541 x 8.069 m centered at origin; the
/// +1.5 m margin keeps wall-adjacent structure.
const BOUNDS: [[f32; 2]; 3] = [
    [-9.8, 9.8],   // x: half 8.27 + margin
    [-0.5, 3.5],   // y: below floor -> structure tops
    [-5.6, 5.6],   // z: half 4.03 + margin
];

#[derive(Debug)]
pub struct GlbMesh {
    pub name: String,
    /// world-space vertex positions
    pub positions: Vec<[f32; 3]>,
    /// triangle indices into `positions`
    pub indices: Vec<[u32; 3]>,
}

#[derive(Deserialize)]
struct GlbJson {
    #[serde(default)]
    nodes: Vec<Node>,
    #[serde(default)]
    meshes: Vec<Mesh>,
    #[serde(default)]
    accessors: Vec<Accessor>,
    #[serde(rename = "bufferViews")]
    buffer_views: Vec<BufferView>,
    #[serde(default)]
    scenes: Vec<Scene>,
    #[serde(default)]
    scene: usize,
}

#[derive(Deserialize)]
struct Node {
    #[serde(default)]
    name: Option<String>,
    #[serde(default)]
    mesh: Option<usize>,
    #[serde(default)]
    children: Vec<usize>,
    #[serde(default)]
    matrix: Option<[f32; 16]>,
    #[serde(default)]
    translation: Option<[f32; 3]>,
    #[serde(default)]
    rotation: Option<[f32; 4]>,
    #[serde(default)]
    scale: Option<[f32; 3]>,
}

#[derive(Deserialize)]
struct Scene {
    #[serde(default)]
    nodes: Vec<usize>,
}

#[derive(Deserialize)]
struct Mesh {
    #[serde(default)]
    primitives: Vec<Primitive>,
}

#[derive(Deserialize)]
struct Primitive {
    #[serde(default)]
    attributes: HashMap<String, usize>,
    #[serde(default)]
    indices: Option<usize>,
}

#[derive(Deserialize)]
struct Accessor {
    #[serde(rename = "bufferView")]
    buffer_view: usize,
    #[serde(default, rename = "byteOffset")]
    byte_offset: usize,
    #[serde(rename = "componentType")]
    component_type: u32,
    #[serde(rename = "count")]
    count: usize,
    #[serde(rename = "type")]
    ty: String,
}

#[derive(Deserialize)]
struct BufferView {
    #[serde(default, rename = "byteOffset")]
    byte_offset: usize,
    #[serde(rename = "byteLength")]
    byte_length: usize,
}

/// Column-major 4x4, glTF convention.
type Mat4 = [[f32; 4]; 4];

fn mat_mul(a: &Mat4, b: &Mat4) -> Mat4 {
    let mut out = [[0.0f32; 4]; 4];
    for c in 0..4 {
        for r in 0..4 {
            out[r][c] = (0..4).map(|k| a[k][c] * b[r][k]).sum();
        }
    }
    out
}

fn mat_apply(m: &Mat4, p: [f32; 3]) -> [f32; 3] {
    [
        m[0][0] * p[0] + m[1][0] * p[1] + m[2][0] * p[2] + m[3][0],
        m[0][1] * p[0] + m[1][1] * p[1] + m[2][1] * p[2] + m[3][1],
        m[0][2] * p[0] + m[1][2] * p[1] + m[2][2] * p[2] + m[3][2],
    ]
}

fn trs_matrix(n: &Node) -> Mat4 {
    if let Some(m) = n.matrix {
        // glTF matrices are column-major flat arrays
        let mut out = [[0.0f32; 4]; 4];
        for c in 0..4 {
            for r in 0..4 {
                out[r][c] = m[c * 4 + r];
            }
        }
        return out;
    }
    let t = n.translation.unwrap_or([0.0; 3]);
    let s = n.scale.unwrap_or([1.0; 3]);
    let q = n.rotation.unwrap_or([0.0, 0.0, 0.0, 1.0]);
    let [x, y, z, w] = q;
    // rotation matrix from quaternion
    let r: Mat4 = [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y + z * w), 2.0 * (x * z - y * w), 0.0],
        [2.0 * (x * y - z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z + x * w), 0.0],
        [2.0 * (x * z + y * w), 2.0 * (y * z - x * w), 1.0 - 2.0 * (x * x + y * y), 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ];
    // row-vector convention: v' = v . S . R . T (scale first, like glTF's
    // column-major T*R*S). M[row][col] = s[row] * r[row][col].
    let mut out = [[0.0f32; 4]; 4];
    for c in 0..3 {
        for row in 0..3 {
            out[row][c] = r[row][c] * s[row];
        }
    }
    out[3][0] = t[0];
    out[3][1] = t[1];
    out[3][2] = t[2];
    out[3][3] = 1.0;
    out
}

fn excluded(name: &str) -> bool {
    let lower = name.to_lowercase();
    EXCLUDE.iter().any(|e| lower.contains(e))
}

/// Parse a .glb file and return collision-suitable meshes (positions +
/// indices, world space, name-filtered). Returns an empty vec if nothing
/// passes the filter.
pub fn load_field_meshes(path: &Path) -> Result<Vec<GlbMesh>> {
    let data = std::fs::read(path)
        .with_context(|| format!("reading GLB {}", path.display()))?;
    if data.len() < 20 || &data[0..4] != b"glTF" {
        bail!("{}: not a GLB file", path.display());
    }
    // chunks: u32 len, u32 type, payload
    let mut off = 12usize;
    let mut json_bytes: Option<&[u8]> = None;
    let mut bin_bytes: Option<&[u8]> = None;
    while off + 8 <= data.len() {
        let len = u32::from_le_bytes(data[off..off + 4].try_into().unwrap()) as usize;
        let ty = u32::from_le_bytes(data[off + 4..off + 8].try_into().unwrap());
        let payload = &data[off + 8..off + 8 + len];
        match ty {
            0x4E4F534A => json_bytes = Some(payload), // 'JSON'
            0x004E4942 => bin_bytes = Some(payload), // 'BIN'
            _ => {}
        }
        off += 8 + len;
    }
    let json = json_bytes.context("GLB has no JSON chunk")?;
    let bin = bin_bytes.context("GLB has no BIN chunk")?;
    let doc: GlbJson = serde_json::from_slice(json).context("parsing GLB JSON chunk")?;

    let read_accessor = |acc: &Accessor| -> Result<(Vec<f32>, usize)> {
        // returns flat f32 data (componentType must be 5126 float32) + comps
        let comps = match acc.ty.as_str() {
            "SCALAR" => 1,
            "VEC2" => 2,
            "VEC3" => 3,
            "VEC4" => 4,
            _ => bail!("unsupported accessor type {}", acc.ty),
        };
        if acc.component_type != 5126 {
            bail!("unsupported componentType {}", acc.component_type);
        }
        let bv = &doc.buffer_views[acc.buffer_view];
        let start = bv.byte_offset + acc.byte_offset;
        let n = acc.count * comps;
        let bytes = &bin[start..start + n * 4];
        let mut out = Vec::with_capacity(n);
        for c in bytes.chunks_exact(4) {
            out.push(f32::from_le_bytes(c.try_into().unwrap()));
        }
        Ok((out, comps))
    };
    let read_indices = |acc: &Accessor| -> Result<Vec<u32>> {
        let bv = &doc.buffer_views[acc.buffer_view];
        let start = bv.byte_offset + acc.byte_offset;
        Ok(match acc.component_type {
            5123 => bin[start..start + acc.count * 2]
                .chunks_exact(2)
                .map(|c| u16::from_le_bytes(c.try_into().unwrap()) as u32)
                .collect(),
            5125 => bin[start..start + acc.count * 4]
                .chunks_exact(4)
                .map(|c| u32::from_le_bytes(c.try_into().unwrap()))
                .collect(),
            t => bail!("unsupported index componentType {t}"),
        })
    };

    let mut meshes = Vec::new();
    // DFS the scene graph carrying world transforms
    fn visit(
        idx: usize,
        parent: &Mat4,
        doc: &GlbJson,
        read_accessor: &dyn Fn(&Accessor) -> Result<(Vec<f32>, usize)>,
        read_indices: &dyn Fn(&Accessor) -> Result<Vec<u32>>,
        meshes: &mut Vec<GlbMesh>,
    ) -> Result<()> {
        let node = &doc.nodes[idx];
        let world = mat_mul(parent, &trs_matrix(node));
        let name = node.name.clone().unwrap_or_else(|| format!("node-{idx}"));
        if let Some(mi) = node.mesh {
            if !excluded(&name) {
                for (pi, prim) in doc.meshes[mi].primitives.iter().enumerate() {
                    let Some(&pos_acc) = prim.attributes.get("POSITION") else {
                        continue;
                    };
                    let acc = &doc.accessors[pos_acc];
                    if acc.count > MAX_VERTS {
                        continue;
                    }
                    let (flat, comps) = read_accessor(acc)?;
                    let positions: Vec<[f32; 3]> = flat
                        .chunks_exact(comps)
                        .map(|c| [c[0], c[1], c[2]])
                        .map(|p| mat_apply(&world, p))
                        .collect();
                    // cull prims fully outside the field envelope (stairs,
                    // bleachers, out-of-bounds staging geometry)
                    if let Some((bmin, bmax)) = positions
                        .iter()
                        .fold(None, |acc: Option<([f32; 3], [f32; 3])>, p| {
                            Some(match acc {
                                None => (*p, *p),
                                Some((lo, hi)) => (
                                    [
                                        lo[0].min(p[0]),
                                        lo[1].min(p[1]),
                                        lo[2].min(p[2]),
                                    ],
                                    [
                                        hi[0].max(p[0]),
                                        hi[1].max(p[1]),
                                        hi[2].max(p[2]),
                                    ],
                                ),
                            })
                        })
                    {
                        let outside = (0..3).any(|i| {
                            bmax[i] < BOUNDS[i][0] || bmin[i] > BOUNDS[i][1]
                        });
                        if outside {
                            continue;
                        }
                    }
                    let indices: Vec<[u32; 3]> = match prim.indices {
                        Some(ii) => {
                            let raw = read_indices(&doc.accessors[ii])?;
                            raw.chunks_exact(3)
                                .map(|c| [c[0], c[1], c[2]])
                                .collect()
                        }
                        // non-indexed: sequential triangles
                        None => (0..positions.len() as u32)
                            .collect::<Vec<_>>()
                            .chunks_exact(3)
                            .map(|c| [c[0], c[1], c[2]])
                            .collect(),
                    };
                    if indices.is_empty() {
                        continue;
                    }
                    meshes.push(GlbMesh {
                        name: format!("{name}#{pi}"),
                        positions,
                        indices,
                    });
                }
            }
        }
        for &c in &node.children {
            visit(c, &world, doc, read_accessor, read_indices, meshes)?;
        }
        Ok(())
    }

    let ident: Mat4 = [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ];
    let roots: Vec<usize> = doc
        .scenes
        .get(doc.scene)
        .map(|s| s.nodes.clone())
        .unwrap_or_default();
    for r in roots {
        visit(
            r,
            &ident,
            &doc,
            &read_accessor,
            &read_indices,
            &mut meshes,
        )?;
    }
    Ok(meshes)
}
