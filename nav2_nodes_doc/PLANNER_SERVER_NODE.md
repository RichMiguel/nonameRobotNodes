# Node: planner_server

Paket Asal: `nav2_planner`  
Executable: `planner_server`  
Peluncur: `nav2_bringup/navigation_launch.py` (via [`robot_bringup.launch.py`](file:///home/acelz/Workspace/nonameRobot/nonameRobotNodes/src/robot_bringup/launch/robot_bringup.launch.py))  
Konfigurasi: [`nav2_params_paper.yaml`](file:///home/acelz/Workspace/nonameRobot/nonameRobotNodes/src/robot_bringup/config/nav2_params_paper.yaml) (Lihat juga: [`PLANNER_SERVER.md`](file:///home/acelz/Workspace/nonameRobot/nonameRobotNodes/nav2_params_doc/PLANNER_SERVER.md))

---

## 1. Ringkasan & Peran
`planner_server` adalah perencana rute global Nav2 (*Global Path Planner*). Pada robot rumah sakit ini, `planner_server` bertugas:
1. Menerima posisi awal dan target tujuan akhir di denah rumah sakit dari BT Navigator.
2. Memuat dan mengevaluasi `global_costmap` (denah lantai statis ditambah rintangan LiDAR).
3. Menggunakan algoritma ekspansi Dijkstra (`nav2_navfn_planner/NavfnPlanner`) untuk menghasilkan jalur optimal terpendek yang menjauh dari dinding koridor.
4. Mempublikasikan rute yang ditemukan ke topik `/plan`.

---

## 2. Antarmuka Topik & Aksi

### Action Server (Antarmuka Utama)
| Nama Action | Tipe Aksi | Deskripsi |
| :--- | :--- | :--- |
| `compute_path_to_pose` | `nav2_msgs/action/ComputePathToPose` | Menerima goal pose dan mengembalikan urutan titik pose jalur global (*Path*). |

### Subscriptions (Masukan)
| Topik | Tipe Pesan | Deskripsi |
| :--- | :--- | :--- |
| `global_costmap/costmap_raw` | `nav2_msgs/msg/Costmap` | Peta biaya rintangan denah global. |

### Publications (Keluaran)
| Topik | Tipe Pesan | Deskripsi |
| :--- | :--- | :--- |
| `/plan` | `nav_msgs/msg/Path` | Garis lintasan global terpendek menuju tujuan (divisualisasikan berupa garis hijau/biru di RViz2). |

---

## 3. Parameter Operasi Kunci
- Frekuensi: `expected_planner_frequency = 1.0 Hz` (hemat CPU, tidak menghitung ulang berlebihan jika jalur masih valid).
- Plugin: `nav2_navfn_planner/NavfnPlanner` (Dijkstra optimal, `use_astar: false`).
