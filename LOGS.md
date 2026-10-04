# Project Development Logs

## [2026-10-03]
- **Setup**: Inisialisasi *workspace* ROS 2 Humble di `/home/pi/workspace` (membuat folder `src` dan menjalankan `colcon build`).
- **Design**: Menetapkan arsitektur *node* berdasarkan framework Nav2.
- **Decision (Controller)**: Memutuskan untuk tetap menggunakan **Regulated Pure Pursuit (RPP)** pada `nav2_controller_server` untuk platform Holonomic X-Drive.
- **Decision (Behavior)**: Menggunakan kombinasi **`nav2_bt_navigator`** untuk *behavior* internal navigasi dan **`fsm_node`** kustom untuk manajemen tugas robot tingkat tinggi.
- **Decision (Documentation)**: Membuat file `DOCS.md` dan `LOGS.md` untuk melacak arsitektur dan riwayat pengembangan proyek.
- **WIP**: Membuat package `base_controller` dan `communication_node.py` menggunakan format frame biner int16 dengan XOR Checksum.
- **Decision (Odometry)**: Memisahkan perhitungan posisi (integrasi X, Y, Theta) ke node kustom tersendiri (`odometry_node`) agar `communication_node` hanya fokus pada I/O Serial berkecepatan tinggi (`/raw_odom_vel`).
- **WIP**: Mengimplementasikan `odometry_node.py` untuk mengintegrasikan kinematika Holonomic X-Drive dan mem-publish TF/Odometry standar Nav2.
- **WIP**: Mendesain dan mengimplementasikan `fsm_node.py` dengan siklus hidup: BOOT -> CHECK -> LISTEN -> CALCULATE -> MOVE -> RECALCULATE -> STOP.
- **WIP**: Membuat package `robot_navigation` (ament_cmake) untuk konfigurasi SLAM Toolbox.
- **WIP**: Membuat package `robot_navigation` (ament_cmake) untuk konfigurasi SLAM Toolbox.
- **Decision (Mapping vs Localization)**: Menetapkan `slam_toolbox` murni untuk fase Mapping saja. Fase Lokalisasi akan menggunakan standar Nav2 (`map_server` dan `amcl`) agar peta berformat `.pgm` bisa diedit secara manual. File lokalisasi SLAM Toolbox dihapus untuk menjaga kebersihan *workspace*.
- **WIP**: Mengonfigurasi parameter Nav2 lengkap (`nav2_params.yaml`) meliputi batasan kecepatan motor, footprint 60x60, RPP, dan toleransi target 10cm.
- **WIP**: Menyusun file master `robot_bringup.launch.py` untuk mengeksekusi Arduino Serial, Odometri, Lidar, Nav2, dan FSM Node secara serentak.
