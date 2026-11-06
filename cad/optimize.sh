#!/bin/bash
# cad/optimize.sh <input.gltf|glb> <output.glb>
# gltf-transform optimize: quantize + meshopt compress + prune unused
# data + merge identical meshes. Keeps named nodes (mechanism parts)
# so animation still works; drops empty transforms.
set -e
IN="$1"; OUT="$2"
npx @gltf-transform/cli optimize "$IN" "$OUT" \
    --compress meshopt --texture-compress webp --flatten \
    --join --simplify 0.9 --weld
echo "optimized -> $OUT ($(du -h "$OUT" | cut -f1))"
