#!/usr/bin/env bash
# ==============================================================================
# Script Penyimpan Peta SLAM Toolbox (Nav2 Map Server & SLAM Posegraph)
#
# Penggunaan:
#   ./save_map.sh [nama_peta]
#
# Contoh:
#   ./save_map.sh lab_map_baru
#
# Output:
#   - <nama_peta>.yaml
#   - <nama_peta>.pgm
#   - <nama_peta>.posegraph & <nama_peta>.data (opsional posegraph SLAM)
# ==============================================================================

set -e

# Nama peta dari argumen pertama, default jika kosong: map_baru
MAP_NAME="${1:-map_baru}"

# Direktori target adalah lokasi folder tempat script ini berada (src/robot_bringup/maps)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET_FILE="$SCRIPT_DIR/$MAP_NAME"

echo "=========================================================="
echo " [SLAM MAPPING] Menyimpan Peta Robot"
echo " Nama File   : $MAP_NAME"
echo " Lokasi Simpan: $TARGET_FILE (.yaml & .pgm)"
echo "=========================================================="

# Sourcing ROS 2 jika belum di-source di sesi saat ini
if [ -z "$ROS_DISTRO" ]; then
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
    fi
fi

# 1. Simpan Peta 2D Occupancy Grid dengan nav2_map_server map_saver_cli
echo "[1/2] Menjalankan nav2_map_server map_saver_cli..."
ros2 run nav2_map_server map_saver_cli -f "$TARGET_FILE" --ros-args -p save_map_timeout:=10.0

# 2. Simpan posegraph SLAM Toolbox (berguna jika ingin reload/lanjut mapping lifelong)
echo "[2/2] Mencoba menyimpan SLAM Toolbox posegraph..."
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$TARGET_FILE'}" > /dev/null 2>&1 || true

echo ""
echo "=========================================================="
echo " Peta BERHASIL disimpan di folder maps!"
echo "=========================================================="
ls -lh "${TARGET_FILE}".* 2>/dev/null || true
echo ""
echo "Tips: Jika ingin langsung memakai peta ini di robot_bringup, jalankan:"
echo "ros2 launch robot_bringup robot_bringup.launch.py map:=$TARGET_FILE.yaml"
echo "=========================================================="
