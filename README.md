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

## 🗺️ Cara Menjalankan Sistem (Step-by-Step)

Untuk menjalankan robot ini, Anda akan membagi tugas antara **Raspberry Pi** (sebagai komputasi utama di robot) dan **Laptop** (sebagai visualisasi jarak jauh). Pastikan keduanya terhubung pada **jaringan Wi-Fi/LAN yang sama**.

### Langkah 1: Persiapan Jaringan (ROS_DOMAIN_ID)
Sistem ROS 2 menggunakan protokol DDS untuk mendeteksi perangkat secara otomatis tanpa perlu IP statis. Kita menggunakan ID **42** agar tidak bertabrakan dengan robot lain.
* **Di Raspberry Pi:** (Sudah terkonfigurasi otomatis di `~/.bashrc`).
* **Di Laptop Ubuntu Anda:** Buka terminal dan jalankan:
  ```bash
  echo 'export ROS_DOMAIN_ID=42' >> ~/.bashrc
  echo 'export ROS_LOCALHOST_ONLY=0' >> ~/.bashrc
  source ~/.bashrc
  ```

### Langkah 2: Menyalakan Robot (Di Raspberry Pi)
Buka terminal (atau via SSH) ke Raspberry Pi Anda, dan jalankan *Master Bringup*. Perintah ini akan menyalakan komunikasi Serial ke Arduino, mengaktifkan Lidar, memuat peta, dan menjalankan AI Navigasi (Nav2):
```bash
ros2 launch robot_navigation robot_bringup.launch.py
```
*Tunggu hingga log menunjukkan pesan bahwa sistem telah aktif dan FSM Node masuk ke status `LISTEN`.*

### Langkah 3: Visualisasi dan Kontrol RViz2 (Di Laptop)
Buka terminal baru di **Laptop** Anda, lalu ikuti langkah ini:
1. Ketik `ros2 topic list`. Jika konfigurasi jaringan Anda benar, Anda akan melihat topik-topik robot bermunculan (seperti `/scan`, `/map`, `/odom`).
2. Jalankan aplikasi RViz2:
   ```bash
   rviz2
   ```
3. **Setup RViz2:**
   * Di panel kiri (*Displays*), ubah **Fixed Frame** menjadi `map`.
   * Klik tombol **Add** di pojok kiri bawah, lalu tambahkan visualisasi berikut:
     - **Map**: Pada menu *Topic*, pilih `/map`. (Untuk melihat peta statis).
     - **Map** (sekali lagi): Pada menu *Topic*, pilih `/global_costmap/costmap`. (Untuk melihat area rintangan Nav2).
     - **LaserScan**: Pada menu *Topic*, pilih `/scan`. (Titik merah dari Lidar).
     - **RobotModel** / **TF**: Untuk melihat posisi dan orientasi robot.
4. **Memberikan Perintah Navigasi:**
   * Klik tombol **2D Goal Pose** di *toolbar* atas RViz2.
   * Klik pada area peta dan tarik kursor untuk menentukan arah hadap (orientasi) tujuan.
   * Raspberry Pi akan menerima perintah tersebut, menghitung rute, dan mengirimkan kecepatan ke Arduino!

## 📄 Struktur Direktori
```text
workspace/
├── src/
│   ├── base_controller/          # (Package Node Python Kustom)
│   ├── robot_navigation/         # (Package Launch & Config)
│   └── sllidar_ros2/             # (Driver Resmi RPLidar A1)
├── nonameRobotKinematic/         # (Source Code Arduino PlatformIO - Git Ignored)
├── DOCS.md                       # (Blueprint Arsitektur)
└── LOGS.md                       # (Log Sejarah Proyek)
```
