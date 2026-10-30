# Topik: /goal_pose

Tipe Pesan: `geometry_msgs/msg/PoseStamped`  
Penerbit (Publishers): `voice_cmd_parser_node`, RViz2 (*Nav2 Goal Button*), Dashboard UI  
Penerima (Subscribers): `fsm_node`, `bt_navigator`  
Frekuensi: Asinkron (Dipublikasikan saat ada perintah misi baru)  
QoS: Volatile, Reliable, Depth: 10

---

## 1. Deskripsi & Peran
Topik `/goal_pose` digunakan untuk menginstruksikan robot menuju posisi dan orientasi target di dalam rumah sakit (seperti kamar pasien, ruang ICU, atau meja perawat). Saat pesan diterima pada topik ini:
1. `fsm_node` menangkap pesan, memvalidasi kesiapan hardware, dan meneruskannya via Action Client `navigate_to_pose`.
2. Jika robot sedang bergerak ke tempat lain, kedatangan pesan baru pada topik ini otomatis membatalkan misi lama dan beralih ke misi baru (*Goal Preemption*).

---

## 2. Struktur Pesan (`geometry_msgs/msg/PoseStamped`)
```yaml
std_msgs/Header header:
  builtin_interfaces/Time stamp
  string frame_id                  # 'map'
geometry_msgs/Pose pose:
  geometry_msgs/Point position:
    float64 x                      # Koordinat X tujuan (meter)
    float64 y                      # Koordinat Y tujuan (meter)
    float64 z                      # 0.0
  geometry_msgs/Quaternion orientation:
    float64 x
    float64 y
    float64 z
    float64 w                      # Arah hadap akhir robot saat sampai
```
