#!/bin/bash
# Script untuk menjalankan NonameRobot Diagnostics Suite (GUI, TUI, CLI)
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 1. Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# 2. Source Workspace jika sudah di-build
if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
    source "$SCRIPT_DIR/install/setup.bash"
fi

# Parsing Argumen
MODE="auto" # "auto", "gui", "tui", "cli"
for arg in "$@"; do
    case "$arg" in
        --tui|-t)
            MODE="tui"
            ;;
        --gui|-g)
            MODE="gui"
            ;;
        --cli|-c)
            MODE="cli"
            ;;
    esac
done

# Cek ketersediaan Display (X11 / Wayland)
HAS_DISPLAY=false
if [ -n "$DISPLAY" ] || [ -n "$WAYLAND_DISPLAY" ]; then
    HAS_DISPLAY=true
fi

echo "=========================================================="
echo "  🤖 NONAME ROBOT - DIAGNOSTICS & TELEMETRY SUITE         "
echo "=========================================================="
echo "Log sesi otomatis disimpan ke:"
echo "  $SCRIPT_DIR/log/diagnostics_sessions/"
echo ""

# Eksekusi sesuai mode
if [ "$MODE" == "gui" ] || ([ "$MODE" == "auto" ] && [ "$HAS_DISPLAY" == true ]); then
    echo "🖥️  Menjalankan Diagnostics Suite dalam Mode GUI (PyQt5)..."
    echo "    (Tip: Gunakan './run_monitor.sh --tui' untuk mode terminal ringan di Raspi)"
    echo "=========================================================="
    python3 "$SCRIPT_DIR/tools/robot_diagnostics_gui.py" "$@"
elif [ "$MODE" == "tui" ] || ([ "$MODE" == "auto" ] && [ "$HAS_DISPLAY" == false ]); then
    echo "📟 Menjalankan Diagnostics Suite dalam Mode TUI (Curses)..."
    echo "    (Ringan < 0.5% CPU, ideal untuk Raspberry Pi over SSH)"
    echo "=========================================================="
    python3 "$SCRIPT_DIR/tools/robot_diagnostics_tui.py" "$@"
else
    echo "💻 Menjalankan Diagnostics dalam mode Log Stream (CLI)..."
    echo "=========================================================="
    python3 "$SCRIPT_DIR/tools/nav2_diagnostics_monitor.py" "$@"
fi
