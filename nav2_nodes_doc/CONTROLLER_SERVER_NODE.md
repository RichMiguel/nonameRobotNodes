# Node: controller_server

Paket Asal: `nav2_controller`  
Executable: `controller_server`  
Peluncur: `nav2_bringup/navigation_launch.py` (via [`robot_bringup.launch.py`](file:///home/acelz/Workspace/nonameRobot/nonameRobotNodes/src/robot_bringup/launch/robot_bringup.launch.py))  
Konfigurasi: [`nav2_params_paper.yaml`](file:///home/acelz/Workspace/nonameRobot/nonameRobotNodes/src/robot_bringup/config/nav2_params_paper.yaml) (Lihat juga: [`CONTROLLER_SERVER.md`](file:///home/acelz/Workspace/nonameRobot/nonameRobotNodes/nav2_params_doc/CONTROLLER_SERVER.md) & [`FOLLOWPATH.md`](file:///home/acelz/Workspace/nonameRobot/nonameRobotNodes/nav2_params_doc/FOLLOWPATH.md))

---

## 1. Ringkasan & Peran
`controller_server` adalah eksekutor kontrol gerak lokal Nav2. Node ini bertugas:
1. Menerima jalur global (*Path*) dari BT Navigator / Planner Server.
2. Memuat dan mengelola plugin algoritma pengikut lintasan (pada robot ini menggunakan **Regulated Pure Pursuit** / `FollowPath`).
3. Memantau rintangan bergerak lokal melalui `local_costmap`.
4. Menghasilkan perintah kecepatan instan pada topik internal `cmd_vel_nav` sebelum dihaluskan oleh `velocity_smoother`.
5. Mengevaluasi kriteria penyelesaian tujuan (`general_goal_checker`) dan deteksi kemacetan (`progress_checker`).

---

## 2. Antarmuka Topik & Aksi

### Action Server (Antarmuka Utama)
| Nama Action | Tipe Aksi | Deskripsi |
| :--- | :--- | :--- |
| `follow_path` | `nav2_msgs/action/FollowPath` | Menerima array titik jalur dan memandu robot mengikutinya. |

### Subscriptions (Masukan)
| Topik | Tipe Pesan | Deskripsi |
| :--- | :--- | :--- |
| `/odom` | `nav_msgs/msg/Odometry` | Umpan balik posisi dan kecepatan robot real-time. |
| `local_costmap/costmap_raw` | `nav2_msgs/msg/Costmap` | Peta rintangan lokal grid. |

### Publications (Keluaran)
| Topik | Tipe Pesan | Deskripsi |
| :--- | :--- | :--- |
| `cmd_vel_nav` (atau `cmd_vel`) | `geometry_msgs/msg/Twist` | Perintah kecepatan sebelum masuk ke velocity smoother. |
| `transformed_global_plan` | `nav_msgs/msg/Path` | Bagian jalur global yang diproyeksikan ke dalam window lokal. |

---

## 3. Parameter Operasi Kunci
- Frekuensi: `controller_frequency = 10.0 Hz`
- Toleransi Tujuan: $10\text{ cm}$ linier dan $5.7^\circ$ rotasi.
- Plugin: Regulated Pure Pursuit dengan kurvatur adaptif ($R_{\text{min}} = 0.90\text{ m}$) dan lookahead $0.75\text{ m}$.
