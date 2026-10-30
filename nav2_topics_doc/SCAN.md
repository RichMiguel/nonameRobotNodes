# Topik: /scan

Tipe Pesan: `sensor_msgs/msg/LaserScan`  
Penerbit (Publisher): `sllidar_node`  
Penerima (Subscribers): `amcl`, `local_costmap`, `global_costmap`, `fsm_node`, RViz2  
Frekuensi Tipikal: $10.0\text{–}12.0\text{ Hz}$  
QoS: Sensor Data (Best Effort, Volatile, Depth: 10)

---

## 1. Deskripsi & Peran
Topik `/scan` adalah saluran data utama sensor persepsi lingkungan 2D. Sensor RPLiDAR memancarkan pulsa laser inframerah yang berputar $360^\circ$ secara kontinu untuk mendeteksi dinding ruangan, pintu, furnitur, tiang infus, dan keberadaan manusia (dokter, perawat, pasien).

---

## 2. Struktur Pesan (`sensor_msgs/msg/LaserScan`)
```yaml
std_msgs/Header header:
  builtin_interfaces/Time stamp
  string frame_id          # 'laser'
float32 angle_min          # -3.14159 rad (-180 deg)
float32 angle_max          #  3.14159 rad (+180 deg)
float32 angle_increment    # Resolusi sudut per sampel (~0.017 rad)
float32 time_increment     # Waktu antar pengukuran
float32 scan_time          # Waktu 1 putaran penuh (~0.1 s = 10 Hz)
float32 range_min          # 0.15 m (filter bodi robot)
float32 range_max          # 12.0 m (jangkauan terjauh)
float32[] ranges           # Array jarak pantulan (meter)
float32[] intensities      # Kekuatan pantulan reflektansi
```

---

## 3. Konsumen Data Topik Ini
1. **AMCL**: Membandingkan pola pantulan `ranges` dengan dinding peta untuk mengoreksi posisi robot.
2. **Costmaps (Local & Global)**: Lapisan `obstacle_layer` menandai sel grid sebagai berpenghalang (*occupied*) jika pantulan $< 3.0\text{ m}$ dan membersihkan sel (*clearing/raytracing*) jika tembus hingga $3.5\text{ m}$.
3. **FSM Node**: Memeriksa bahwa frekuensi `/scan` aktif sebagai syarat kelayakan sistem sebelum memulai pergerakan otonom.
