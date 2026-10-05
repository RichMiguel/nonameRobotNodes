#!/bin/bash

# Konfigurasi Path
WORKSPACE_DIR="/home/pi/workspace"
MAPS_DIR="$WORKSPACE_DIR/src/robot_navigation/maps"

# Fungsi untuk menampilkan header
print_header() {
    clear
    echo "=================================================="
    echo "       🤖 NONAME ROBOT CONTROL PANEL 🤖       "
    echo "=================================================="
    echo ""
}

# Fungsi untuk memilih peta
select_map() {
    echo "Peta yang tersedia di direktori maps/:"
    # List semua file .yaml
    map_files=($(ls $MAPS_DIR/*.yaml 2>/dev/null | xargs -n 1 basename))
    
    if [ ${#map_files[@]} -eq 0 ]; then
        echo "⚠️ Tidak ada peta (.yaml) yang ditemukan!"
        return 1
    fi

    for i in "${!map_files[@]}"; do
        echo "[$i] ${map_files[$i]}"
    done

    read -p "Pilih nomor peta [0]: " map_idx
    map_idx=${map_idx:-0}

    SELECTED_MAP="${map_files[$map_idx]}"
    echo "✅ Memilih peta: $SELECTED_MAP"
    sleep 1
    return 0
}

# Menu Utama
while true; do
    print_header
    echo "1. 🗺️  Mulai Mapping (SLAM Toolbox)"
    echo "2. 💾 Simpan Peta Hasil Mapping"
    echo "3. 🚀 Navigasi Normal (Production)"
    echo "4. 💻 Simulasi / Test Navigasi (Hardware OFF)"
    echo "5. 🛠️  Build Ulang Sistem (colcon build)"
    echo "0. ❌ Keluar"
    echo ""
    read -p "Pilih menu (0-5): " choice

    case $choice in
        1)
            print_header
            echo "Memulai SLAM Toolbox..."
            echo "Gunakan RViz2 di laptop untuk memandu robot berkeliling."
            source $WORKSPACE_DIR/install/setup.bash
            ros2 launch robot_navigation slam_mapping.launch.py
            ;;
        2)
            print_header
            read -p "Masukkan nama file peta baru (tanpa ekstensi): " map_name
            if [ -n "$map_name" ]; then
                echo "Menyimpan peta ke: $MAPS_DIR/$map_name"
                source /opt/ros/humble/setup.bash
                ros2 run nav2_map_server map_saver_cli -f "$MAPS_DIR/$map_name"
                echo "✅ Peta berhasil disimpan!"
            else
                echo "❌ Nama peta tidak boleh kosong!"
            fi
            sleep 3
            ;;
        3)
            print_header
            if select_map; then
                echo "MENGAKTIFKAN HARDWARE & NAVIGASI..."
                source $WORKSPACE_DIR/install/setup.bash
                ros2 launch robot_navigation robot_bringup.launch.py map:="$MAPS_DIR/$SELECTED_MAP"
            fi
            ;;
        4)
            print_header
            if select_map; then
                echo "MENGAKTIFKAN SIMULASI (HARDWARE BYPASS)..."
                source $WORKSPACE_DIR/install/setup.bash
                ros2 launch robot_navigation robot_bringup.launch.py map:="$MAPS_DIR/$SELECTED_MAP" ignore_hardware:=true
            fi
            ;;
        5)
            print_header
            echo "Membangun ulang (Build) Workspace..."
            cd $WORKSPACE_DIR
            colcon build --symlink-install
            echo "✅ Build selesai!"
            sleep 2
            ;;
        0)
            echo "Keluar dari Control Panel. Sampai jumpa!"
            exit 0
            ;;
        *)
            echo "❌ Pilihan tidak valid!"
            sleep 1
            ;;
    esac
done
