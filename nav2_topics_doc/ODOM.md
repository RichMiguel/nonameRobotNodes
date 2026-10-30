# Topik: /odom

Tipe Pesan: `nav_msgs/msg/Odometry`  
Penerbit (Publisher): `odometry_node`  
Penerima (Subscribers): `amcl`, `controller_server`, `fsm_node`, `velocity_smoother`, RViz2  
Frekuensi Tipikal: $20.0\text{ Hz}$ / $50.0\text{ Hz}$  
QoS: Volatile, Reliable, Depth: 10

---

## 1. Deskripsi & Peran
Topik `/odom` memuat informasi navigasi kinematika lengkap robot terhadap frame lokal `odom`:
1. **Pose Kumulatif**: Posisi 3D $(X, Y, Z)$ dan orientasi dalam format Quaternion $(x, y, z, w)$ yang dihitung dari integrasi kecepatan roda menggunakan metode Midpoint Runge-Kutta.
2. **Twist Instan**: Kecepatan linier dan sudut saat ini.
3. **Kovariansi**: Matriks kepastian kovariansi $6\times 6$ untuk pose dan twist (digunakan oleh filter AMCL untuk memperkirakan ketidakpastian gerak robot).

---

## 2. Struktur Pesan (`nav_msgs/msg/Odometry`)
```yaml
std_msgs/Header header:
  builtin_interfaces/Time stamp
  string frame_id                  # 'odom'
string child_frame_id              # 'base_link'
geometry_msgs/PoseWithCovariance pose:
  geometry_msgs/Pose pose:
    geometry_msgs/Point position   # x, y, z
    geometry_msgs/Quaternion orientation # x, y, z, w
  float64[36] covariance           # Kovariansi x, y, z, roll, pitch, yaw
geometry_msgs/TwistWithCovariance twist:
  geometry_msgs/Twist twist:
    geometry_msgs/Vector3 linear   # vx, vy, vz
    geometry_msgs/Vector3 angular  # wx, wy, wz
  float64[36] covariance
```

---

## 3. Matriks Kovariansi yang Digunakan
- Sumbu Aktif ($X, Y, \text{Yaw}$): $\sigma^2 = 0.001$ (variansi kecil karena odometri deterministik roda 45mm presisi).
- Sumbu Terkunci ($Z, \text{Roll}, \text{Pitch}$): $\sigma^2 = 1.000.000,0$ (variansi sangat besar, menandakan robot beroperasi murni di atas bidang datar 2D).
