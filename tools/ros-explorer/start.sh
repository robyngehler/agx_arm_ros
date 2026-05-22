#!/usr/bin/env bash
# start.sh – Launch ROS Explorer (scanner API + Vite dev server)
#
# Usage:  ./start.sh [/path/to/ros_workspace]
#         ./start.sh --build   # build production bundle, then serve on :7357

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_PATH="${1:-$(cd "$SCRIPT_DIR/../.." && pwd)}"

# ── production build mode ─────────────────────────────────────────────────────
if [[ "${1:-}" == "--build" ]]; then
    echo "Building frontend…"
    cd "$SCRIPT_DIR"
    npm run build
    WS_PATH="${2:-$(cd "$SCRIPT_DIR/../.." && pwd)}"
    echo "Starting scanner server on http://localhost:7357"
    python3 scanner/ros_scanner.py "$WS_PATH" --serve
    exit 0
fi

# ── dev mode: scanner API + Vite dev server ───────────────────────────────────
echo "Starting ROS Explorer in dev mode"
echo "  Workspace: $WS_PATH"
echo "  Scanner API: http://localhost:7357/api/scan"
echo "  UI (dev):    http://localhost:5173/"
echo ""
echo "Press Ctrl+C to stop."
echo ""

# Kill any stale processes from a previous session
lsof -ti:7357 2>/dev/null | xargs kill -9 2>/dev/null || true
lsof -ti:5173 2>/dev/null | xargs kill -9 2>/dev/null || true
sleep 0.5

# Start scanner in background
python3 "$SCRIPT_DIR/scanner/ros_scanner.py" "$WS_PATH" --serve --port 7357 &
SCANNER_PID=$!

cleanup() {
    kill "$SCANNER_PID" 2>/dev/null || true
}
trap cleanup EXIT

# Start Vite dev server
cd "$SCRIPT_DIR"
npm run dev
