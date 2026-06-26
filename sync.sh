#!/bin/bash
# =============================================================================
# sync.sh - Pull the latest project files from Windows (OneDrive) into the
#           WSL working copy. Run this from WSL whenever files change on the
#           Windows side (e.g. after Claude edits them).
#
# Usage (from anywhere in WSL):
#   bash ~/Projects/self-balancing-robot/sync.sh
# =============================================================================
set -euo pipefail

# Windows source (note the space and Cyrillic in the path - keep it quoted)
SRC="/mnt/c/Users/artur/OneDrive/Документи/Claude/Projects/self-balancing-robot"

# WSL destination (the local working copy used for arduino-cli)
DST="$HOME/Projects/self-balancing-robot"

if [ ! -d "$SRC" ]; then
  echo "ERROR: Windows source not found:"
  echo "  $SRC"
  echo "If this path is in OneDrive, make sure the folder is 'Always keep on this device'"
  echo "(cloud-only files are not readable through /mnt/c)."
  exit 1
fi

mkdir -p "$DST"

echo "Syncing:"
echo "  FROM  $SRC"
echo "  TO    $DST"
echo ""

if command -v rsync >/dev/null 2>&1; then
  # -r recurse, -t keep times, -v verbose, --delete mirror removals,
  # --modify-window for FAT/NTFS timestamp slop. Skips git/build cruft.
  rsync -rtv --modify-window=2 --delete \
    --exclude='.git/' \
    --exclude='build/' \
    --exclude='*.tmp' \
    "$SRC"/ "$DST"/
else
  echo "(rsync not installed - falling back to cp; run 'sudo apt install rsync' for faster/mirrored sync)"
  cp -r "$SRC"/. "$DST"/
fi

echo ""
echo "Done. Sketches available in WSL:"
# List each sketch folder that contains a matching .ino
for ino in "$DST"/*/*.ino "$DST"/*.ino; do
  [ -e "$ino" ] && echo "  $ino"
done

echo ""
echo "To build the v2 controller:"
echo "  cd $DST/balance_v2"
echo "  arduino-cli compile --fqbn arduino:avr:uno ."
echo "  arduino-cli upload  --fqbn arduino:avr:uno --port /dev/ttyUSB0 ."
