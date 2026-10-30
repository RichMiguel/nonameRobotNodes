# Topik: Nav2 Action Interfaces

Mekanisme Aksi (ROS 2 Actions) menggunakan 5 sub-topik komunikasi standar per aksi:
- `_action/send_goal` (`Goal`)
- `_action/cancel_goal` (`Cancel`)
- `_action/get_result` (`Result`)
- `_action/feedback` (`Feedback`)
- `_action/status` (`Status`)

---

## 1. Daftar Aksi Navigasi Utama

### A. `navigate_to_pose`
- **Tipe Aksi**: `nav2_msgs/action/NavigateToPose`
- **Server**: `bt_navigator`
- **Klien**: `fsm_node`, RViz2 (*Navigate To Pose tool*)
- **Peran**: Menerima koordinat tujuan akhir di peta, mengeksekusi perencanaan rute global, pelacakan jalur, dan penanganan pemulihan hingga robot berhenti di tujuan.
- **Feedback**: Jarak sisa ke tujuan (meter), estimasi waktu tempuh (detik), jumlah pemulihan yang dieksekusi.

---

### B. `follow_path`
- **Tipe Aksi**: `nav2_msgs/action/FollowPath`
- **Server**: `controller_server`
- **Klien**: `bt_navigator`
- **Peran**: Mengeksekusi rute lokal yang diberikan oleh planner server.
- **Feedback**: Jarak yang telah ditempuh dan kecepatan saat ini.

---

### C. `compute_path_to_pose`
- **Tipe Aksi**: `nav2_msgs/action/ComputePathToPose`
- **Server**: `planner_server`
- **Klien**: `bt_navigator`
- **Peran**: Menghitung jalur optimal Dijkstra dari titik robot saat ini menuju target.
- **Result**: Objek `nav_msgs/msg/Path` berisi array titik-titik koordinat.

---

### D. Perilaku Pemulihan (`spin`, `backup`, `wait`)
- **Tipe Aksi**: `nav2_msgs/action/Spin`, `BackUp`, `Wait`
- **Server**: `behavior_server`
- **Klien**: `bt_navigator`
- **Peran**: Mengeksekusi aksi darurat ketika robot terperangkap atau jalan tertutup sementara.
