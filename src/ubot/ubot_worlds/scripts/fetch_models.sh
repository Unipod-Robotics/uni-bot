#!/usr/bin/env bash
# Download the Gazebo Harmonic ports of the AWS RoboMaker house / bookstore / warehouse models.
#
# The models (~170 MB of meshes) are not committed. They come from tb_worlds/models in
# github.com/zp78-ship-it/turtlebot-maze (MIT, Sebastian Castro et al.), which packages the
# community Harmonic ports of the AWS RoboMaker worlds (originally MIT-0, aws-robotics). The commit
# is pinned so every benchmark run uses identical assets.
#
# Usage: ubot_worlds/scripts/fetch_models.sh        (then rebuild ubot_worlds)
set -euo pipefail

REPO=https://github.com/zp78-ship-it/turtlebot-maze.git
COMMIT=30424d56102e8dfac7f56d6b7302c3490d68b2bb

PKG_DIR=$(cd "$(dirname "$0")/.." && pwd)
DEST="$PKG_DIR/models_external"
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

if [ -f "$DEST/.commit" ] && [ "$(cat "$DEST/.commit")" = "$COMMIT" ]; then
  echo "models_external already at $COMMIT"
  exit 0
fi

git -C "$TMP" init -q
git -C "$TMP" remote add origin "$REPO"
git -C "$TMP" config core.sparseCheckout true
echo "tb_worlds/models/" > "$TMP/.git/info/sparse-checkout"
echo "LICENSE" >> "$TMP/.git/info/sparse-checkout"
git -C "$TMP" fetch -q --depth 1 --filter=blob:none origin "$COMMIT"
git -C "$TMP" checkout -q FETCH_HEAD

rm -rf "$DEST"
mkdir -p "$DEST"
cp -r "$TMP"/tb_worlds/models/aws_robomaker_* "$DEST"/
cp "$TMP/LICENSE" "$DEST/LICENSE.turtlebot-maze"
echo "$COMMIT" > "$DEST/.commit"
echo "Fetched $(ls "$DEST" | grep -c aws_robomaker_) models into $DEST ($(du -sh "$DEST" | cut -f1))"
