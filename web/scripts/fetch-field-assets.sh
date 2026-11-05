#!/usr/bin/env bash
# Fetch the 2026 REBUILT field visual assets (issue #42).
#
# Downloads the official AdvantageScope field package
# Field3d_2026FRCFieldV1 from Mechanical-Advantage/AdvantageScopeAssets and
# installs the GLBs under web/public/field-assets/ (gitignored). The sim's
# physics never touches these — they are visuals only; colliders live in
# fields/rebuilt.toml. See web/ASSETS.md for provenance/licensing.
set -euo pipefail

ZIP_URL="https://github.com/Mechanical-Advantage/AdvantageScopeAssets/releases/download/archive-v1/Field3d_2026FRCFieldV1.zip"
OUT_DIR="$(cd "$(dirname "$0")/.." && pwd)/public/field-assets"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

echo "downloading $ZIP_URL"
curl -fSL "$ZIP_URL" -o "$TMP/field.zip"
unzip -o -q "$TMP/field.zip" -d "$TMP/field"

mkdir -p "$OUT_DIR"
# model.glb    = full field (staged Fuel meshes hidden at render time)
# model_0.glb  = single Fuel game piece (kept for future piece rendering)
# config.json  = AdvantageScope asset config (AprilTags, dims reference)
cp "$TMP/field/model.glb"   "$OUT_DIR/rebuilt-2026.glb"
cp "$TMP/field/model_0.glb" "$OUT_DIR/fuel.glb"
cp "$TMP/field/config.json" "$OUT_DIR/advantagescope-config.json"

echo "installed field assets to $OUT_DIR"
ls -la "$OUT_DIR"
