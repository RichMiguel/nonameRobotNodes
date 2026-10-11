#!/usr/bin/env python3
"""
Pipeline Konversi 2D Map ke Gazebo Harmonic World (.world)
Secara otomatis:
1. Membaca file peta 2D (.yaml dan gambar .png/.pgm)
2. Mengonversi kontur dinding menjadi model SDF 1.10 dengan Gazebo Harmonic plugins:
   - Physics (DART)
   - UserCommands (create entity spawner)
   - SceneBroadcaster (GUI/rendering)
   - Sensors (GPU LiDAR ogre2)
   - Directional Sun Light
3. Menganalisis area bebas untuk menentukan titik spawn robot yang aman (terjauh dari dinding).
4. Menyinkronkan file ke direktori install (jika diperlukan).
5. Mencetak perintah 'ros2 launch' siap pakai.
"""

import os
import sys
import argparse
import yaml
import numpy as np
from PIL import Image

def find_workspace_root():
    cur = os.path.dirname(os.path.abspath(__file__))
    while cur and cur != os.path.dirname(cur):
        if os.path.isdir(os.path.join(cur, 'src')):
            return cur
        cur = os.path.dirname(cur)
    return os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))

def calculate_safe_spawn(map_yaml_path):
    """
    Menghitung titik spawn yang aman (di dalam ruangan dan jauh dari dinding)
    menggunakan distance transform.
    """
    with open(map_yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    img_rel = data['image']
    img_dir = os.path.dirname(os.path.abspath(map_yaml_path))
    img_path = os.path.join(img_dir, img_rel)

    if not os.path.exists(img_path):
        return None, None, None

    img = Image.open(img_path).convert('L')
    arr = np.array(img)
    res = float(data.get('resolution', 0.05))
    origin = data.get('origin', [0.0, 0.0, 0.0])

    # Di ROS map: 0 = occupied (dinding), 254/255 = free (lantai), ~205 = unknown
    free_mask = (arr > 220)
    
    # Hitung jarak dari setiap pixel bebas ke rintangan/dinding terdekat
    try:
        from scipy.ndimage import distance_transform_edt
        dist_map = distance_transform_edt(free_mask)
        
        # Cari batas dinding aktif untuk menghindari area void di luar ruangan
        occ_rows, occ_cols = np.where(arr == 0)
        if len(occ_rows) > 0:
            min_r, max_r = occ_rows.min(), occ_rows.max()
            min_c, max_c = occ_cols.min(), occ_cols.max()
            # Buat mask area di dalam bounding box dinding
            bbox_mask = np.zeros_like(dist_map, dtype=bool)
            bbox_mask[min_r:max_r, min_c:max_c] = True
            dist_map_inner = np.where(bbox_mask, dist_map, 0)
            best_idx = np.unravel_index(np.argmax(dist_map_inner), dist_map.shape)
        else:
            best_idx = np.unravel_index(np.argmax(dist_map), dist_map.shape)

        r, c = best_idx
        clearance_m = dist_map[r, c] * res

        # Konversi koordinat pixel ke koordinat dunia ROS/Gazebo
        H, _ = arr.shape
        spawn_x = origin[0] + c * res
        spawn_y = origin[1] + (H - 1 - r) * res
        return round(float(spawn_x), 2), round(float(spawn_y), 2), round(float(clearance_m), 2)
    except Exception as e:
        print(f"Peringatan: Gagal menghitung auto-spawn ({e})")
        return None, None, None

def check_spawn_clearance(map_yaml_path, x, y):
    """Mengecek apakah koordinat (x, y) berada di area bebas atau mengenai dinding."""
    with open(map_yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    img_rel = data['image']
    img_dir = os.path.dirname(os.path.abspath(map_yaml_path))
    img_path = os.path.join(img_dir, img_rel)
    if not os.path.exists(img_path):
        return None

    img = Image.open(img_path).convert('L')
    arr = np.array(img)
    res = float(data.get('resolution', 0.05))
    origin = data.get('origin', [0.0, 0.0, 0.0])
    H, W = arr.shape

    c = int(round((x - origin[0]) / res))
    r = int(round(H - 1 - ((y - origin[1]) / res)))

    if 0 <= r < H and 0 <= c < W:
        val = arr[r, c]
        if val == 0:
            return "TABRAKAN_DINDING"
        elif val > 200:
            return "BEBAS"
        else:
            return "UNKNOWN"
    return "DILUAR_PETA"

def main():
    parser = argparse.ArgumentParser(description="Pipeline Konversi 2D Map ROS ke Gazebo Harmonic World")
    parser.add_argument("-m", "--map", default="", help="Path ke file peta .yaml")
    parser.add_argument("-o", "--output", default="", help="Path output file .world")
    parser.add_argument("--wall-height", type=float, default=2.0, help="Tinggi dinding (meter, default: 2.0)")
    parser.add_argument("--wall-thickness", type=float, default=0.1, help="Ketebalan dinding (meter, default: 0.1)")
    parser.add_argument("--spawn-x", type=float, default=None, help="Koordinat spawn robot X (meter)")
    parser.add_argument("--spawn-y", type=float, default=None, help="Koordinat spawn robot Y (meter)")
    parser.add_argument("--spawn-yaw", type=float, default=0.0, help="Orientasi spawn robot Yaw (radian, default: 0.0)")

    args = parser.parse_args()
    ws_root = find_workspace_root()

    # Default map path jika tidak ditentukan
    if not args.map:
        default_map = os.path.join(ws_root, "src", "robot_bringup", "maps", "lab_map.yaml")
        if os.path.exists(default_map):
            args.map = default_map
        else:
            print("Error: Harap tentukan path file map dengan --map <path/to/map.yaml>")
            sys.exit(1)

    args.map = os.path.abspath(args.map)
    if not os.path.exists(args.map):
        print(f"Error: File peta tidak ditemukan di {args.map}")
        sys.exit(1)

    # Tentukan output world path
    map_base = os.path.splitext(os.path.basename(args.map))[0]
    if not args.output:
        args.output = os.path.join(ws_root, "src", "robot_gazebo", "worlds", f"{map_base}.world")
    args.output = os.path.abspath(args.output)
    os.makedirs(os.path.dirname(args.output), exist_ok=True)

    print("=" * 60)
    print("🚀 MEMULAI PIPELINE KONVERSI 2D MAP -> GAZEBO HARMONIC WORLD")
    print("=" * 60)
    print(f"📄 Input Map        : {args.map}")
    print(f"🌍 Output World      : {args.output}")
    print(f"📏 Tinggi Dinding   : {args.wall_height} m")
    print(f"📐 Tebal Dinding    : {args.wall_thickness} m")

    # Jalankan generator 2d_to_world
    try:
        # Import MapToGazeboSegments dari 2d_to_world
        pkg_path = os.path.join(ws_root, "src", "2d_map_to_gazebo_world", "2d_to_world")
        if pkg_path not in sys.path:
            sys.path.append(pkg_path)

        import importlib
        m2g_mod = importlib.import_module("2d_to_world.2d_to_world")
        MapToGazeboSegments = m2g_mod.MapToGazeboSegments

        import rclpy
        from rclpy.parameter import Parameter
        rclpy.init(args=None)
        params = [
            Parameter('map_path', Parameter.Type.STRING, args.map),
            Parameter('output_world', Parameter.Type.STRING, args.output),
            Parameter('wall_height', Parameter.Type.DOUBLE, float(args.wall_height)),
            Parameter('wall_thickness', Parameter.Type.DOUBLE, float(args.wall_thickness))
        ]
        node = MapToGazeboSegments(parameter_overrides=params)
        node.destroy_node()
        rclpy.shutdown()
        print("✅ Pembuatan model SDF dan penyisipan Gazebo Harmonic Plugins BERHASIL.")
    except Exception as e:
        print(f"❌ Terjadi kesalahan saat konversi: {e}")
        sys.exit(1)

    # Sinkronisasi ke install space jika target berada di src/robot_gazebo/worlds/
    install_worlds_dir = os.path.join(ws_root, "install", "robot_gazebo", "share", "robot_gazebo", "worlds")
    if os.path.exists(install_worlds_dir):
        install_target = os.path.join(install_worlds_dir, os.path.basename(args.output))
        if not os.path.exists(install_target) or not os.path.samefile(args.output, install_target):
            import shutil
            shutil.copyfile(args.output, install_target)
            print(f"🔄 Tersinkronisasi ke install space: {install_target}")

    # Analisis dan rekomendasi titik spawn
    auto_x, auto_y, clearance = calculate_safe_spawn(args.map)
    chosen_x = args.spawn_x if args.spawn_x is not None else (auto_x if auto_x is not None else 2.45)
    chosen_y = args.spawn_y if args.spawn_y is not None else (auto_y if auto_y is not None else 1.80)

    status = check_spawn_clearance(args.map, chosen_x, chosen_y)
    print("\n" + "-" * 60)
    print("📍 ANALISIS TITIK SPAWN ROBOT:")
    print(f"   Koordinat Spawn Terpilih : x = {chosen_x}, y = {chosen_y}, yaw = {args.spawn_yaw}")
    if status == "BEBAS":
        print(f"   Status Lokasi           : ✅ AMAN (Berada di lantai bebas)")
    elif status == "TABRAKAN_DINDING":
        print(f"   Status Lokasi           : ⚠️ PERINGATAN! Titik ini mengenai DINDING.")
        if auto_x is not None:
            print(f"   Rekomendasi Aman        : x = {auto_x}, y = {auto_y} (Jarak ke dinding: {clearance}m)")
            chosen_x, chosen_y = auto_x, auto_y
    else:
        print(f"   Status Lokasi           : {status}")

    print("-" * 60)
    print("\n🎉 PIPELINE SELESAI! Anda dapat langsung menjalankan simulasi dengan perintah:")
    print("\n" + "=" * 60)
    print(f"ros2 launch robot_gazebo gazebo_bringup.launch.py \\")
    print(f"    map:={args.map} \\")
    print(f"    world:={args.output} \\")
    print(f"    spawn_x:={chosen_x} \\")
    print(f"    spawn_y:={chosen_y} \\")
    print(f"    spawn_yaw:={args.spawn_yaw}")
    print("=" * 60 + "\n")

if __name__ == "__main__":
    main()
