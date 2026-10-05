# Noname Robot Nodes (ROS 2 Humble)

Repositori ini berisi *high-level control* untuk robot holonomic (X-Drive) berbasis ROS 2 Humble. Sistem ini berjalan di atas Raspberry Pi 4 dan berkomunikasi dengan Arduino Mega (PlatformIO) melalui protokol Serial (UART).

## 🚀 Fitur Utama
1. **Navigasi Otonom (Nav2):** Menggunakan AMCL, Global/Local Costmaps, dan Regulated Pure Pursuit (RPP) Controller.
2. **Mapping (SLAM Toolbox):** Pembangunan peta ruangan 2D secara *real-time*.
3. **Komunikasi Biner Native (Little-Endian):** Pengiriman *payload* Serial ultra-cepat tanpa *parsing string*, sinkron 100% dengan prosesor AVR Arduino Mega.
4. **Finite State Machine (FSM):** Penanganan status robot secara mandiri (BOOT, CHECK, LISTEN, CALCULATE, MOVE).
5. **Hardware Bypass Mode:** Mode simulasi/pengujian navigasi tanpa perlu menyambungkan sensor Lidar dan *driver* motor fisik.

---

## 🛠️ Instalasi & Persiapan

### 1. Dependensi (Di Raspberry Pi)
Pastikan Anda sudah menginstal paket-paket berikut:
```bash
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-tf-transformations python3-transforms3d
```

### 2. Konfigurasi Jaringan DDS
Untuk memantau robot menggunakan laptop, pastikan Raspberry Pi dan Laptop Ubuntu Anda berada di jaringan Wi-Fi yang sama, lalu setel `ROS_DOMAIN_ID` di **kedua perangkat** (misal ID = 42).

```bash
echo 'export ROS_DOMAIN_ID=42' >> ~/.bashrc
echo 'export ROS_LOCALHOST_ONLY=0' >> ~/.bashrc
source ~/.bashrc
```

---

## 🎮 Cara Penggunaan: Control Panel Interaktif

Untuk memudahkan operasional, kami telah menyediakan antarmuka terminal interaktif. Anda tidak perlu lagi menghafal perintah ROS 2 yang panjang.

Cukup jalankan perintah ini di root *workspace* Raspberry Pi Anda:
```bash
cd ~/workspace
./start_robot.sh
```

**Menu yang tersedia:**
1. **Mulai Mapping (SLAM Toolbox):** Robot akan mengaktifkan Lidar dan membangun peta (*rviz2* digunakan untuk *teleop* / pergerakan).
2. **Simpan Peta Hasil Mapping:** Menyimpan peta `.yaml` dan `.pgm` langsung ke dalam folder `src/robot_navigation/maps/`.
3. **Navigasi Normal (Production):** Mengaktifkan komunikasi Arduino, Lidar, dan AI Nav2 menggunakan peta yang dipilih.
4. **Simulasi / Test Navigasi (Hardware OFF):** Mem-Bypass Lidar dan Serial. Mode ini akan menerbitkan *Fake TF* (`map -> odom -> base_link`) sehingga Anda bisa memonitor peta dan alur rute (*Path Planning*) di RViz2 dari laptop meskipun robot tidak dirakit.
5. **Build Ulang Sistem:** Memanggil `colcon build --symlink-install`.

---

## 💻 Setup RViz2 (Visualisasi di Laptop)

Saat robot sedang dalam mode **Navigasi**, ikuti langkah berikut di Laptop Anda:
1. Buka terminal baru dan ketik `rviz2`.
2. Ubah **Fixed Frame** menjadi `map`.
3. Klik tombol **Add** (kiri bawah) -> tab **By topic**:
   - Cari `/map` -> pilih **Map**. *(Penting: Buka panah pengaturan Map di kiri layar, dan pastikan **Durability Policy** disetel ke `Transient Local` agar peta muncul).*
   - Cari `/global_costmap/costmap` -> pilih **Map** (ubah *Color Scheme* ke `costmap`).
   - Cari `/scan` -> pilih **LaserScan** (titik rintangan aktual).
4. Klik tombol **Add** -> tab **By display type** -> pilih **TF** (untuk melihat posisi robot).
5. **Mulai Navigasi:** Gunakan tombol **2D Goal Pose** di *toolbar* atas untuk memberikan perintah bergerak!

---

## 📄 Logika Perbaikan Terkini
* **`base_controller`**: Berisi *node* Python. 
  - `communication_node.py` telah disinkronkan ke **Little-Endian** (`<hhh`) sesuai bitshift Arduino.
  - `fsm_node.py` mendukung mode `bypass_health_check`.
* **`robot_navigation`**: Berisi launch file dan konfigurasi.
  - `nav2_params.yaml` disesuaikan untuk standar sintaks **ROS 2 Humble** (menggunakan garis miring `/` pada nama *plugin*).
  - Parameter Behavior Tree (*BT XML*) dikembalikan ke konfigurasi sistem bawaan.
* **`start_robot.sh`**: *Dashboard* utama pengembang.
