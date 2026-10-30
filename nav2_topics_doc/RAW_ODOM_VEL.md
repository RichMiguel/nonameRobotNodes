# Topik: raw_odom_vel

Tipe Pesan: `geometry_msgs/msg/TwistStamped`  
Penerbit (Publisher): `communication_node`  
Penerima (Subscriber): `odometry_node`  
Frekuensi Tipikal: $20.0\text{ Hz}$  
QoS: Volatile, Reliable, Depth: 10

---

## 1. Deskripsi & Peran
Topik `raw_odom_vel` membawa data kecepatan aktual sasis robot yang diukur secara fisik dari putaran roda motor (PG45) melalui mikrokontroler Arduino Mega. Data ini diekstrak dari frame biner serial dan distempel waktu (*timestamped*) saat diterima oleh komputer utama.

---

## 2. Struktur Pesan (`geometry_msgs/msg/TwistStamped`)
```yaml
std_msgs/Header header:
  builtin_interfaces/Time stamp  # Waktu penerimaan paket serial
  string frame_id                # 'base_link'
geometry_msgs/Twist twist:
  geometry_msgs/Vector3 linear   # x: Vx aktual (m/s), y: Vy aktual (m/s)
  geometry_msgs/Vector3 angular  # z: Wz aktual (rad/s)
```

---

## 3. Rantai Data
1. Encoder motor membaca pulsa $\rightarrow$ `actualRPM` deterministik (tanpa filter EMA).
2. Forward Kinematics di Arduino menghitung $V_x, V_y, \omega$.
3. Dikirim via frame serial `0xBB 0x66` ke `communication_node`.
4. Dipublikasikan ke `raw_odom_vel` untuk diintegrasikan oleh `odometry_node`.
