# Node: behavior_server

Paket Asal: `nav2_behaviors`  
Executable: `behavior_server`  
Peluncur: `nav2_bringup/navigation_launch.py` (via [`robot_bringup.launch.py`](file:///home/acelz/Workspace/nonameRobot/nonameRobotNodes/src/robot_bringup/launch/robot_bringup.launch.py))  
Konfigurasi: [`nav2_params_paper.yaml`](file:///home/acelz/Workspace/nonameRobot/nonameRobotNodes/src/robot_bringup/config/nav2_params_paper.yaml) (Lihat juga: [`BEHAVIOR_SERVER.md`](file:///home/acelz/Workspace/nonameRobot/nonameRobotNodes/nav2_params_doc/BEHAVIOR_SERVER.md))

---

## 1. Ringkasan & Peran
`behavior_server` adalah server perilaku pemulihan otonom (*Recovery Behaviors*) Nav2. Node ini bertugas:
1. Menyediakan aksi pemulihan saat robot terjebak, tersesat, atau menemui lorong yang buntu.
2. Mengeksekusi aksi gerak khusus:
   - `spin`: Memutar bodi robot di tempat secara perlahan untuk menyapu LiDAR dan memperbarui peta lokal.
   - `backup`: Mundur lurus beberapa puluh sentimeter untuk memberi ruang bebas di depan robot.
   - `wait`: Berhenti diam sejenak menunggu rintangan bergerak (seperti perawat/pasien) lewat.
3. Memastikan setiap manuver pemulihan aman dari benturan menggunakan proyeksi lintasan pada `local_costmap`.

---

## 2. Antarmuka Action Server

| Nama Action | Tipe Aksi | Deskripsi |
| :--- | :--- | :--- |
| `spin` | `nav2_msgs/action/Spin` | Memutar robot dengan kecepatan sudut maksimal $1.0\text{ rad/s}$ dan akselerasi $1.0\text{ rad/s}^2$. |
| `backup` | `nav2_msgs/action/BackUp` | Menggerakkan robot mundur perlahan secara terkontrol. |
| `wait` | `nav2_msgs/action/Wait` | Memerintahkan robot berhenti diam selama durasi tertentu. |

---

## 3. Subscriptions & Publications
- **Subscriptions**: `local_costmap/costmap_raw`, `/odom`.
- **Publications**: `cmd_vel` (saat aksi recovery aktif mengambil alih kendali gerak).
