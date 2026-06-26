#!/bin/bash
# =============================================================================
# flash.sh - One-command build pipeline for the balancing robot (run in WSL).
#
# Automates the manual steps:
#   1. attach the CH340 USB device from Windows into WSL (usbipd)
#   2. give your user read/write on the serial port
#   3. sync latest files from Windows (OneDrive) into the WSL copy
#   4. compile the sketch
#   5. upload to the Arduino
#   6. (optional) open the serial monitor
#
# Examples:
#   bash flash.sh --all              # attach + perms + sync + compile + upload
#   bash flash.sh --all --monitor    # ...then open serial monitor
#   bash flash.sh --compile          # just compile
#   bash flash.sh --upload --monitor # upload then monitor
#   bash flash.sh --sketch gy --all  # build a different sketch folder
# =============================================================================
set -euo pipefail

# ---- Config (override with env vars or flags) ------------------------------
HWID="${HWID:-1a86:7523}"                 # CH340 USB hardware id (usbipd)
PORT="${PORT:-/dev/ttyUSB0}"              # serial port in WSL
FQBN="${FQBN:-arduino:avr:uno}"
BAUD="${BAUD:-115200}"
SKETCH_NAME="${SKETCH_NAME:-balance_v2}"  # which sketch folder to build

# Windows source (keep quoted - has a space and Cyrillic)
WIN_SRC="/mnt/c/Users/artur/OneDrive/Документи/Claude/Projects/self-balancing-robot"
# WSL working copy (fast local build target)
WSL_DST="${WSL_DST:-$HOME/Projects/self-balancing-robot}"

# ---- Flags -----------------------------------------------------------------
do_attach=0; do_perms=0; do_sync=0; do_compile=0; do_upload=0; do_monitor=0
while [ $# -gt 0 ]; do
  case "$1" in
    --attach)  do_attach=1 ;;
    --perms)   do_perms=1 ;;
    --sync)    do_sync=1 ;;
    --compile) do_compile=1 ;;
    --upload)  do_upload=1 ;;
    --monitor) do_monitor=1 ;;
    --all)     do_attach=1; do_perms=1; do_sync=1; do_compile=1; do_upload=1 ;;
    --sketch)  SKETCH_NAME="$2"; shift ;;
    --port)    PORT="$2"; shift ;;
    --baud)    BAUD="$2"; shift ;;
    -h|--help)
      sed -n '2,25p' "$0"; exit 0 ;;
    *) echo "Unknown option: $1 (try --help)"; exit 1 ;;
  esac
  shift
done
# Default action if none given
if [ $((do_attach+do_perms+do_sync+do_compile+do_upload+do_monitor)) -eq 0 ]; then
  do_sync=1; do_compile=1; do_upload=1
fi

SKETCH_DIR="$WSL_DST/$SKETCH_NAME"

step() { echo ""; echo "=== $* ==="; }

# List sketch folders (a dir whose name matches a .ino inside it) in WSL_DST.
list_sketches() {
  local d name found=0
  for d in "$WSL_DST"/*/; do
    [ -d "$d" ] || continue
    name="$(basename "$d")"
    if [ -e "$d/$name.ino" ]; then echo "  --sketch $name"; found=1; fi
  done
  [ "$found" = 1 ] || echo "  (none found - did you run --sync yet?)"
}

# Fail early with a helpful list if the chosen sketch is missing/invalid.
require_sketch() {
  if [ ! -e "$SKETCH_DIR/$SKETCH_NAME.ino" ]; then
    echo "ERROR: sketch '$SKETCH_NAME' not found."
    echo "       expected: $SKETCH_DIR/$SKETCH_NAME.ino"
    echo "Available sketches in $WSL_DST:"
    list_sketches
    exit 1
  fi
}

# ---- 1. Attach USB (Windows -> WSL) ----------------------------------------
if [ "$do_attach" = 1 ]; then
  step "Attaching USB $HWID into WSL"
  if command -v usbipd.exe >/dev/null 2>&1; then
    # Tolerate "already attached"; bind (one-time) needs an ELEVATED PowerShell:
    #   usbipd bind --hardware-id 1a86:7523
    usbipd.exe attach --wsl --hardware-id "$HWID" \
      || echo "(attach skipped - already attached, or run 'usbipd bind --hardware-id $HWID' once in an admin PowerShell)"
  else
    echo "usbipd.exe not found from WSL. Run this in Windows PowerShell instead:"
    echo "  usbipd attach --wsl --hardware-id $HWID"
  fi
  # Wait for the port to enumerate
  for i in $(seq 1 10); do [ -e "$PORT" ] && break; sleep 0.5; done
fi

# ---- 2. Serial port permissions --------------------------------------------
if [ "$do_perms" = 1 ]; then
  step "Granting access to $PORT"
  if [ -e "$PORT" ]; then
    sudo chmod a+rw "$PORT"
  else
    echo "WARNING: $PORT not present yet (is the device attached?)."
  fi
fi

# ---- 3. Sync Windows -> WSL ------------------------------------------------
if [ "$do_sync" = 1 ]; then
  step "Syncing project from Windows"
  if [ ! -d "$WIN_SRC" ]; then
    echo "ERROR: Windows source not found ($WIN_SRC)."
    echo "If it's in OneDrive, set the folder to 'Always keep on this device'."
    exit 1
  fi
  mkdir -p "$WSL_DST"
  if command -v rsync >/dev/null 2>&1; then
    rsync -rt --modify-window=2 --delete \
      --exclude='.git/' --exclude='build/' --exclude='*.tmp' \
      "$WIN_SRC"/ "$WSL_DST"/
  else
    echo "(rsync not installed; using cp. 'sudo apt install rsync' is faster.)"
    cp -r "$WIN_SRC"/. "$WSL_DST"/
  fi
  echo "Synced to $WSL_DST"
fi

# Validate the sketch up front if we're going to build/flash it.
if [ "$do_compile" = 1 ] || [ "$do_upload" = 1 ]; then
  require_sketch
fi

# ---- 4. Compile ------------------------------------------------------------
if [ "$do_compile" = 1 ]; then
  step "Compiling $SKETCH_NAME"
  arduino-cli compile --fqbn "$FQBN" "$SKETCH_DIR"
fi

# ---- 5. Upload -------------------------------------------------------------
if [ "$do_upload" = 1 ]; then
  step "Uploading $SKETCH_NAME to $PORT"
  [ -e "$PORT" ] || { echo "ERROR: $PORT not found (attach + perms first)."; exit 1; }
  arduino-cli upload -p "$PORT" --fqbn "$FQBN" "$SKETCH_DIR"
fi

# ---- 6. Monitor ------------------------------------------------------------
if [ "$do_monitor" = 1 ]; then
  step "Serial monitor on $PORT @ $BAUD (Ctrl+C to exit)"
  arduino-cli monitor -p "$PORT" --config baudrate="$BAUD"
fi

echo ""
echo "Done."
