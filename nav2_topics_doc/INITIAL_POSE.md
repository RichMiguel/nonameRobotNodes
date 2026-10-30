# Topik: /initialpose

Tipe Pesan: `geometry_msgs/msg/PoseWithCovarianceStamped`  
Penerbit (Publishers): RViz2 (*2D Pose Estimate*), Script Inisialisasi Otomatis  
Penerima (Subscriber): `amcl`  
Frekuensi: Asinkron (Dipublikasikan sekali saat startup / penentuan titik awal robot)  
QoS: Volatile, Reliable, Depth: 10

---

## 1. Deskripsi & Peran
Topik `/initialpose` digunakan untuk memberikan perkiraan posisi awal robot di denah rumah sakit kepada filter lokalisasi `amcl`. Begitu pesan ini diterima:
1. `amcl` akan mereset awan partikelnya (*particle cloud*).
2. Ribuan partikel disebarkan mengelilingi koordinat perkiraan awal tersebut dengan radius sebaran sesuai matriks kovariansi yang ditentukan.
3. Begitu robot bergerak sedikit dan pindaian laser LiDAR masuk, partikel langsung mengelompok (*konvergen*) mengunci posisi akurat robot.

---

## 2. Struktur Pesan (`geometry_msgs/msg/PoseWithCovarianceStamped`)
```yaml
std_msgs/Header header:
  builtin_interfaces/Time stamp
  string frame_id                  # 'map'
geometry_msgs/PoseWithCovariance pose:
  geometry_msgs/Pose pose:
    geometry_msgs/Point position   # x, y, z
    geometry_msgs/Quaternion orientation # x, y, z, w
  float64[36] covariance           # Kovariansi awal perkiraan posisi & rotasi
```
