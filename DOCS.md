# Robot Project Documentation

## Architecture Overview
Proyek robot berbasis ROS 2 Humble menggunakan Raspberry Pi 4B (8GB RAM) sebagai pemroses utama (High-Level) dan Arduino Mega sebagai pengontrol level rendah (Low-Level).

### Hardware & Kinematics
- **SBC**: Raspberry Pi 4B (8GB RAM)
- **Microcontroller**: Arduino Mega
- **Drive Type**: Holonomic X-Drive (Kinematik ditangani oleh Arduino)
- **Sensors**: RPLidar A1

### ROS 2 Nodes Blueprint
1. **`lidar_node`**: Menggunakan *package* `sllidar_ros2` untuk menangani data dari RPLidar A1 menjadi topik `/scan`.
2. **`slam_node`**: Dikelola melalui package `robot_bringup` menggunakan `slam_toolbox`. Hanya digunakan khusus saat **fase pemetaan (Mapping)** untuk menghasilkan file peta statis (`.pgm` dan `.yaml`).
3. **`nav2_map_server` & `nav2_amcl`**: Digunakan saat **fase navigasi/operasi normal**. `map_server` memuat peta statis yang sudah dibersihkan, dan `amcl` melakukan lokalisasi presisi tinggi menggunakan partikel filter.
4. **`nav2_costmap_2d`**: Standard Nav2 costmap (Global dan Local) untuk menentukan area aman dan rintangan.
5. **`nav2_planner_server`**: Standard Nav2 *planner* untuk mencari rute (path) dari titik A ke titik B.
6. **`nav2_controller_server`**: Menggunakan algoritma **Regulated Pure Pursuit (RPP)** untuk mengikuti jalur yang telah dibuat.
7. **`nav2_velocity_smoother`**: Menangani akselerasi dan deselerasi target kecepatan (`/cmd_vel` menjadi `/cmd_vel_smoothed`) agar pergerakan robot lebih halus.
8. **`nav2_bt_navigator`**: Behavior Tree bawaan Nav2 untuk mengatur alur logika navigasi, interaksi *planner*, *controller*, dan *recovery behaviors*.
9. **`fsm_node`**: Node kustom Python sebagai otak utama robot (Task Manager). Mengatur logika aplikasi tingkat tinggi dan mengirim *goal* ke `nav2_bt_navigator`.
10. **`communication_node`**: Node kustom Python untuk menjembatani komunikasi Serial. Meneruskan `cmd_vel` ke Arduino (int16), dan membaca kecepatan mentah odometri (Vx, Vy, Wz) dari Arduino, lalu mem-publish-nya ke `/raw_odom_vel`.
11. **`odometry_node`**: Node kustom Python untuk mengintegrasikan (menghitung) posisi X, Y, dan Theta dari data kecepatan `/raw_odom_vel`, kemudian mem-publish topik standar `/odom` dan Transformasi (TF) `odom -> base_link`.

### Workspace Packages
- **`robot_base`** (ament_python): Berisi *core script* Python seperti FSM, Komunikasi Serial, dan Odometri.
- **`robot_bringup`** (ament_cmake): Integrator utama sistem, memuat *launch file* modular (`base.launch.py`, `lidar.launch.py`, `robot_bringup.launch.py`), parameter Nav2, dan peta.
- **`robot_gazebo`** (ament_cmake): Lingkungan terisolasi yang khusus digunakan untuk simulasi 3D di Gazebo.
- **`sllidar_ros2`**: Driver dari pihak ketiga untuk sensor RPLidar.
