# Topik: /cmd_vel

Tipe Pesan: `geometry_msgs/msg/Twist`  
Penerbit (Publishers): `velocity_smoother` (Nav2 otonom), `teleop_wasd_node` (Manual)  
Penerima (Subscribers): `communication_node`  
Frekuensi Tipikal: $10.0\text{ Hz}$  
QoS: Volatile, Best Effort / Reliable, Depth: 10

---

## 1. Deskripsi & Peran
Topik `/cmd_vel` adalah kanal perintah kecepatan gerak utama robot. Topik ini membawa vektor kecepatan linier 3-sumbu ($V_x, V_y, V_z$) dan kecepatan sudut 3-sumbu ($\omega_x, \omega_y, \omega_z$). Pada robot planar holonomik ini:
- `linear.x`: Kecepatan maju ($+$) / mundur ($-$) dalam meter per detik ($\text{m/s}$).
- `linear.y`: Kecepatan geser menyamping / *strafe* ke kiri ($+$) / kanan ($-$) dalam meter per detik ($\text{m/s}$).
- `angular.z`: Kecepatan putar sudut rotasi berlawanan jarum jam / CCW ($+$) / searah jarum jam / CW ($-$) dalam radian per detik ($\text{rad/s}$).
- `linear.z`, `angular.x`, `angular.y` bernilai $0.0$ karena robot beroperasi di lantai 2D.

---

## 2. Struktur Definisi Pesan (`geometry_msgs/msg/Twist`)
```yaml
geometry_msgs/Vector3 linear:
  float64 x   # m/s
  float64 y   # m/s
  float64 z   # 0.0
geometry_msgs/Vector3 angular:
  float64 x   # 0.0
  float64 y   # 0.0
  float64 z   # rad/s
```

---

## 3. Batas Operasi Klinis (Hospital Safety Limits)
Berdasarkan `nav2_params_paper.yaml` (`velocity_smoother`):
- $V_x$ maks: $0.30\text{ m/s}$ (maju), $-0.30\text{ m/s}$ (mundur).
- $V_y$ maks: $\pm 0.01\text{ m/s}$ (dibatasi kecil untuk stabilitas arah RPP).
- $\omega_z$ maks: $\pm 1.0\text{ rad/s}$ ($\approx 57.3^\circ/\text{s}$).
