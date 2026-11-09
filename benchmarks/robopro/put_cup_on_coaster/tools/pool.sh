#!/usr/bin/env bash
# Start the episode worker pool on this GPU node, outside Slurm (nice 10), detached.
#   bash tools/pool.sh SLOTS [SLURM_NODE_NAME]
# Logs: .pool/supervisor-<host>.log and .pool/logs/<worker>.log. Stop: python tools/pool.py stop
set -euo pipefail
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source "$HERE/env.sh"
SLOTS=${1:-8}; NODE=${2:-}; GUEST=${GUEST_SLOTS:-0}; START_GB=${START_GB:-40}; MIN_GB=${MIN_GB:-20}
MAX_TEMP=${MAX_TEMP:-89}
mkdir -p "$HERE/.pool"
cd "$HERE"
setsid nice -n 10 "$SIM_PY" tools/pool.py worker --slots "$SLOTS" --guest-slots "$GUEST" --start-gb "$START_GB" \
  --min-gb "$MIN_GB" --max-temp "$MAX_TEMP" ${NODE:+--slurm-node "$NODE"} \
  >> "$HERE/.pool/supervisor-$(hostname).log" 2>&1 < /dev/null &
echo "pool supervisor pid $! on $(hostname): $SLOTS slots"
