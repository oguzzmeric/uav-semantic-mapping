#!/usr/bin/env bash
# Apply this project's changes to a PX4-Autopilot checkout (idempotent).
#   - applies setup/px4_patches/*.patch (skips ones already applied)
#   - links sim/px4/airframes/* into PX4's posix airframes directory
#   - links sim/gz_models/* into PX4's Gazebo models directory (PX4 spawns from there)
# Usage: setup/px4_overlay.sh [PX4_DIR]   (default: ~/PX4-Autopilot)
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${1:-$HOME/PX4-Autopilot}"
AIRFRAMES_DIR="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"
GZ_MODELS_DIR="$PX4_DIR/Tools/simulation/gz/models"

[ -d "$PX4_DIR/.git" ] || { echo "PX4 checkout not found: $PX4_DIR" >&2; exit 1; }

for patch in "$REPO_DIR"/setup/px4_patches/*.patch; do
	name="$(basename "$patch")"
	if git -C "$PX4_DIR" apply --check --reverse "$patch" 2>/dev/null; then
		echo "[skip]  $name (already applied)"
	elif git -C "$PX4_DIR" apply --check "$patch" 2>/dev/null; then
		git -C "$PX4_DIR" apply "$patch"
		echo "[apply] $name"
	else
		echo "[FAIL]  $name does not apply cleanly to $PX4_DIR" >&2
		exit 1
	fi
done

for airframe in "$REPO_DIR"/sim/px4/airframes/*; do
	ln -sfn "$airframe" "$AIRFRAMES_DIR/$(basename "$airframe")"
	echo "[link]  $(basename "$airframe")"
done

for model in "$REPO_DIR"/sim/gz_models/*/; do
	model="${model%/}"
	ln -sfn "$model" "$GZ_MODELS_DIR/$(basename "$model")"
	echo "[link]  gz model $(basename "$model")"
done

echo "Done. Re-run cmake so new gz_* targets appear: make -C $PX4_DIR px4_sitl"
