# Topik: /plan dan transformed_global_plan

Tipe Pesan: `nav_msgs/msg/Path`  
Penerbit (Publishers): `planner_server` (menghasilkan `/plan`), `controller_server` (menghasilkan `transformed_global_plan`)  
Penerima (Subscribers): `controller_server`, RViz2  
Frekuensi: $1.0\text{ Hz}$ (`/plan`), $10.0\text{ Hz}$ (`transformed_global_plan`)  
QoS: Volatile, Reliable, Depth: 10

---

## 1. Deskripsi & Peran
1. **`/plan`**: Lintasan global lengkap yang dihitung dari posisi robot saat ini menyusuri koridor rumah sakit hingga ke depan pintu kamar target. Rute ini dihitung oleh algoritma Dijkstra pada `planner_server`.
2. **`transformed_global_plan`**: Potongan lokal dari `/plan` yang dipotong dan ditransformasikan ke frame lokal `odom` di dalam jendela $3\text{m} \times 3\text{m}$ (`local_costmap`). Lintasan lokal inilah yang dilacak secara real-time oleh controller Regulated Pure Pursuit (`FollowPath`).

---

## 2. Struktur Pesan (`nav_msgs/msg/Path`)
```yaml
std_msgs/Header header:
  builtin_interfaces/Time stamp
  string frame_id                  # 'map' untuk /plan, 'odom' untuk transformed plan
geometry_msgs/PoseStamped[] poses  # Array titik-titik koordinat lintasan berurutan
```
