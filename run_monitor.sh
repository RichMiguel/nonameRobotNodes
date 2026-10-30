#!/bin/bash
# Script untuk menjalankan Nav2 Diagnostics & Timing Monitor
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 1. Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# 2. Source Workspace jika sudah di-build
if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
    source "$SCRIPT_DIR/install/setup.bash"
fi

echo "=========================================================="
echo "  🚀 MEMULAI NAV2 TIMING & CAUSE-EFFECT DIAGNOSTICS...    "
echo "=========================================================="
echo "Log sesi akan otomatis disimpan ke:"
echo "  $SCRIPT_DIR/log/diagnostics_sessions/"
echo ""
echo "Tekan Ctrl+C kapan saja untuk berhenti dan menyimpan data."
echo "=========================================================="
sleep 1

python3 "$SCRIPT_DIR/tools/nav2_diagnostics_monitor.py" "$@"
