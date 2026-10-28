#!/usr/bin/env bash
# Download the textures of the village world (idempotent), into sim/gz_worlds/textures/
# (gitignored).
#
# Ground: ambientCG materials, CC0 (https://ambientcg.com). Only the 2K color map of each
# is kept: ~1 cm per texel on the 20 m ground tiles, finer than the camera's 1.8 cm/px at 30 m.
# Cars: the three Fuel hatchback textures (CC0), renamed. Gazebo caches textures by file
# name and all three are called hatchback.png, so every car would get the first one.
#
# Usage: setup/fetch_textures.sh
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT_DIR="$REPO_DIR/sim/gz_worlds/textures"
MATERIALS=(Grass004 Ground037 Asphalt031)
FUEL=https://fuel.gazebosim.org/1.0/OpenRobotics/models
declare -A FUEL_TEXTURES=(
	[hatchback_white.png]="$FUEL/Hatchback/3/files/materials/textures/hatchback.png"
	[hatchback_blue.png]="$FUEL/Hatchback%20blue/5/files/materials/textures/hatchback.png"
	[hatchback_red.png]="$FUEL/Hatchback%20red/4/files/materials/textures/hatchback.png"
)

mkdir -p "$OUT_DIR"
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

for material in "${MATERIALS[@]}"; do
	target="$OUT_DIR/$material.jpg"
	if [ -f "$target" ]; then
		echo "[skip]  $material (already there)"
		continue
	fi
	curl -fsSL -o "$tmp/$material.zip" "https://ambientcg.com/get?file=${material}_2K-JPG.zip"
	unzip -q -j -o "$tmp/$material.zip" "*_Color.jpg" -d "$tmp/$material"
	mv "$tmp/$material"/*_Color.jpg "$target"
	echo "[fetch] $material -> ${target#"$REPO_DIR"/}"
done

for name in "${!FUEL_TEXTURES[@]}"; do
	target="$OUT_DIR/$name"
	if [ -f "$target" ]; then
		echo "[skip]  $name (already there)"
		continue
	fi
	curl -fsSL -o "$tmp/$name" "${FUEL_TEXTURES[$name]}"
	mv "$tmp/$name" "$target"
	echo "[fetch] $name -> ${target#"$REPO_DIR"/}"
done
