# Field visual assets (issue #42)

The web client renders the real 2026 REBUILT field model when the GLB is
present at `web/public/field-assets/rebuilt-2026.glb`. If it is missing the
client falls back to the primitive walls/obstacles described by the field
spec — physics is unaffected either way (colliders are authoritative in
`fields/rebuilt.toml`, not in the mesh).

## Provenance

| File | Source |
| --- | --- |
| `rebuilt-2026.glb` | `model.glb` from the AdvantageScope asset package **`Field3d_2026FRCFieldV1`** |
| `fuel.glb` | `model_0.glb` from the same package (single Fuel piece) |
| `advantagescope-config.json` | `config.json` from the same package (AprilTag layout, dims) |

- Package URL:
  `https://github.com/Mechanical-Advantage/AdvantageScopeAssets/releases/download/archive-v1/Field3d_2026FRCFieldV1.zip`
- The field model was created by **@Juniormunk** for AdvantageScope
  (Mechanical-Advantage), matching FIRST's official 2026 field CAD. This is
  the same asset fieldcore (`DDguan2010/fieldcore`) ships.
- The AdvantageScopeAssets repo does not declare an SPDX license
  (`license: other` on the GitHub API); the assets are distributed publicly
  for FRC visualization use. We fetch at build time rather than vendoring
  the ~18 MB binary into this repo; the download URL above is the canonical
  upstream release.

## Fetching

```sh
cd web
npm run fetch-assets   # wraps scripts/fetch-field-assets.sh
```

Requires `curl` + `unzip`. `web/public/field-assets/` is gitignored.

## Geometry conventions

The GLB is in **meters, y-up**: x runs along the field length (carpet spans
x ≈ ±11.3 m, walls at ±8.27), z along the width (±4.03), y up. That matches
the sim's world frame 1:1, so the client applies no transform. Staged
`GE-26900: Fuel` meshes embedded in the field model are hidden at load time
(they are decor, not simulated pieces).

Field-element bounding boxes were measured from the model's node AABBs and
encoded as the primitive colliders in `fields/rebuilt.toml`:

| Element | GLB center (x, y, z) | Approx size (m) |
| --- | --- | --- |
| Hub base | (±3.645, —, 0) | 1.18 × 1.18, rim y≈2.13 |
| Trench | (±3.645, —, ±2.6) skirts + arm to z≈4.1 | ~1.2 × ~0.7 × 1.0 |
| Bump | (±3.645, 0.08, ±1.67) | 0.69 × 0.16 × 2.16 |
| Tower | (±7.2..8.27, —, ∓0.29) | ~1.1 × 1.83 × 1.25 |
| Depot rails | (±7.85, —, ∓1.93) | ~0.75 × 0.05 × 1.0 |

`tests/field_config.rs` pins these numbers so the TOML cannot silently
drift from the model.

## Robot visual assets (issue #43)

Archetype TOMLs may carry a `[model]` table (`url`, `scale`, `offset`,
`rotation_deg`). The viz config message forwards it per robot id; the web
client renders the GLB parented to the chassis when it loads and keeps the
primitive box otherwise. Physics is unaffected — the cuboid in
`half_extents` is always the collider.

The shipped archetypes reference the **2026 KitBot**:

| File | Source |
| --- | --- |
| `kitbot-2026.glb` | `model.glb` from the AdvantageScope asset package **`Robot_2026FRCKitBotV2`** |
| `kitbot-2026-config.json` | `config.json` from the same package (pose fix, camera nodes) |

- Package URL:
  `https://github.com/Mechanical-Advantage/AdvantageScopeAssets/releases/download/archive-v1/Robot_2026FRCKitBotV2.zip`
- The KitBot model ships in AdvantageScope (Mechanical-Advantage) for FRC
  visualization; same provenance/licensing notes as the field above —
  fetched at build time, never vendored.
- The archetype `[model]` `rotation_deg = [90, 0, 90]` and
  `offset = [-0.30, 0.0, 0.05]` mirror the package's `config.json`
  (`rotations: x+90°, z+90°`; `position: [-0.3, 0, 0.05]`), which maps the
  GLB into the chassis frame (+x forward, +y up, meters).

The intake region, when an archetype declares one, renders as a
translucent chassis-relative box — a v1 mechanism proxy, not an
articulated mechanism.
