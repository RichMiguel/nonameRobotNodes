# NoName Robot Nodes

Proyek ROS 2 Humble untuk robot bergerak otomatis (Autonomous Mobile Robot) berbasis **Holonomic X-Drive**. Proyek ini menggunakan **Raspberry Pi 4B** sebagai prosesor utama (*High-Level*) untuk menjalankan Navigation 2 (Nav2) Stack, dan **Arduino Mega** sebagai *Low-Level Controller* penggerak motor.

## 🌟 Ikhtisar Arsitektur

Sistem ini didesain secara modular, tangguh, dan efisien untuk komunikasi berkecepatan tinggi dengan mikrokontroler. 

* **Drive Type:** Holonomic X-Drive (Mecanum / Omni wheels)
* **SBC:** Raspberry Pi 4B (8GB RAM)
* **Microcontroller:** Arduino Mega (menangani PID, Inverse/Forward Kinematics)
* **Sensor Utama:** RPLidar A1
* **Framework:** ROS 2 Humble + Nav2 + SLAM Toolbox

## 🧩 ROS 2 Packages & Nodes

Proyek ini terbagi menjadi dua *package* kustom utama di dalam *workspace*:

### 1. `base_controller`
Package Python kustom yang menjembatani komunikasi ke *hardware* dan logika *state machine*.
* **`communication_node.py`**: Jembatan Serial (UART) ke Arduino. Menggunakan protokol frame biner kustom (Header `0xAA 0x55`), algoritma **XOR Checksum** untuk integritas data, dan **Fixed-Point Scaling** (Float ke int16 dikalikan 1000) untuk menghemat 50% *bandwidth* serial. Node ini meneruskan `/cmd_vel` ke Arduino dan menerima *raw velocities* ($V_x, V_y, \omega_z$) dari Arduino.
* **`odometry_node.py`**: Node integrasi kinematik Holonomic. Mengambil *raw velocities*, menghitung (integrasi) posisi $X, Y, \Theta$, lalu mempublikasikan `/odom` beserta Transformasi TF (`odom` -> `base_link`).
* **`fsm_node.py`**: Otak utama (*Brain*) robot menggunakan pola *Finite State Machine*. Siklus hidupnya adalah:
  `BOOT` -> `CHECK` (memastikan Lidar, Odom, & Nav2 siap) -> `LISTEN` -> `CALCULATE` -> `MOVE` -> `RECALCULATE` (Recovery) -> `STOP`.

### 2. `robot_navigation`
Package berbasis `ament_cmake` untuk menampung file konfigurasi dan peluncuran (Launch) ekosistem standar ROS 2.
* **Nav2 Config** (`nav2_params.yaml`): Konfigurasi komprehensif untuk *AMCL*, *Costmaps* (Footprint 60x60 cm), algoritma *Regulated Pure Pursuit* (RPP), dan *Velocity Smoother* (Limit $1.0$ m/s).
* **SLAM Config** (`mapper_params_online_async.yaml`): Parameter untuk menjalankan SLAM Toolbox saat membuat peta awal.
* **Master Launch** (`robot_bringup.launch.py`): Menjalankan seluruh sistem (*Base Controller*, Lidar, dan Nav2) secara serentak.

## 🚀 Instalasi & Kompilasi (Build)

1. **Persiapan Dependencies**
   Pastikan Anda telah menginstal ROS 2 Humble dan package pendukung Nav2 & Lidar:
   ```bash
   sudo apt update
   sudo apt install ros-humble-nav2-bringup ros-humble-sllidar-ros2 ros-humble-slam-toolbox
   ```

2. **Clone Repository & Build**
   ```bash
   mkdir -p ~/workspace/src
   cd ~/workspace/src
   git clone https://github.com/<YOUR_USERNAME>/nonameRobotNodes.git .
   
   cd ~/workspace
   colcon build --symlink-install
   source install/setup.bash
   ```

## 🗺️ Cara Penggunaan (Usage)

### Fase 1: Memetakan Ruangan (Mapping)
Gunakan peluncuran SLAM Toolbox untuk membuat peta ruangan baru secara manual.
```bash
ros2 launch robot_navigation slam_mapping.launch.py
```
Setelah peta di RViz terlihat utuh, simpan peta dengan menjalankan (di terminal baru):
```bash
ros2 run nav2_map_server map_saver_cli -f ~/workspace/src/robot_navigation/maps/peta_ruangan
```

### Fase 2: Navigasi Otonom (Operasi Normal)
Gunakan peluncuran master (*Master Bringup*) yang akan menyalakan komunikasi Arduino, Odometri, Lidar, dan Nav2 (Map Server + AMCL) menggunakan peta yang sudah disimpan.
```bash
ros2 launch robot_navigation robot_bringup.launch.py
```
Setelah itu, pastikan di log bahwa FSM Node telah masuk ke state `LISTEN`. Anda bisa memberikan target kordinat Navigasi dari RViz2 (Gunakan tombol *2D Goal Pose*).

## 📄 Struktur Direktori
```text
workspace/
├── src/
│   ├── base_controller/          # (Package Node Python Kustom)
│   └── robot_navigation/         # (Package Launch & Config)
├── DOCS.md                       # (Dokumentasi Rinci Blueprint Arsitektur)
└── LOGS.md                       # (Log Sejarah Pengembangan Proyek)
```
