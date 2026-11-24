#!/usr/bin/env bash
# Usage: bash tools/probe_bag.sh <path/to/...__simulation.bag.zip>
# Unzips the bag next to the zip, runs probe_bag.py inside the simulator image, writes a
# "<run>__probe" folder (a few MB), then deletes the unzipped bag again.
set -euo pipefail
ZIP="$(cd "$(dirname "$1")" && pwd)/$(basename "$1")"
DIR="$(dirname "$ZIP")"
BASE="$(basename "$ZIP" __simulation.bag.zip)"
TOOLS="$(cd "$(dirname "$0")" && pwd)"
WORK="$DIR/${BASE}__probe"
mkdir -p "$WORK"
echo "Unzipping $BASE ..."
unzip -o -q "$ZIP" simulation.bag -d "$WORK"
cp "$TOOLS/probe_bag.py" "$WORK/probe_bag.py"
docker run --rm -v "$WORK":/work krpc/intball-simulator:latest bash -c \
  "source /opt/ros/melodic/setup.bash && python /work/probe_bag.py /work/simulation.bag /work"
rm -f "$WORK/simulation.bag" "$WORK/probe_bag.py"
ls -la "$WORK"
