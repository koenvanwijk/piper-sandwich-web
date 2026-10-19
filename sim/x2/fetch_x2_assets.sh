#!/usr/bin/env bash
# Fetch the official AgiBot X2 URDF + meshes (Mulan PSL v2) at a pinned commit.
set -euo pipefail
REV=575cc6b988f976c23550e0db85aa1e5475d3652d
DST="$(cd "$(dirname "$0")" && pwd)/assets/X2"
TMP=$(mktemp -d); trap 'rm -rf "$TMP"' EXIT
git clone -q https://github.com/AgibotTech/agibot_x2_urdf "$TMP/x2" && git -C "$TMP/x2" checkout -q "$REV"
mkdir -p "$DST/meshes"
cp "$TMP/x2/X2_URDF-v1.4.0/X2-Ultra_omnipicker.urdf" "$DST/"
grep -o 'filename="[^"]*"' "$DST/X2-Ultra_omnipicker.urdf" | sed 's/filename="\.\///;s/"//' | sort -u \
  | while read -r f; do cp "$TMP/x2/X2_URDF-v1.4.0/$f" "$DST/$f"; done
echo "X2 assets in $DST"
