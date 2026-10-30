# Node: bt_navigator

Paket Asal: `nav2_bt_navigator`  
Executable: `bt_navigator`  
Peluncur: `nav2_bringup/navigation_launch.py` (via [`robot_bringup.launch.py`](file:///home/acelz/Workspace/nonameRobot/nonameRobotNodes/src/robot_bringup/launch/robot_bringup.launch.py))  
Konfigurasi: [`nav2_params_paper.yaml`](file:///home/acelz/Workspace/nonameRobot/nonameRobotNodes/src/robot_bringup/config/nav2_params_paper.yaml) (Lihat juga: [`BT_NAVIGATOR.md`](file:///home/acelz/Workspace/nonameRobot/nonameRobotNodes/nav2_params_doc/BT_NAVIGATOR.md))

---

## 1. Ringkasan & Peran
`bt_navigator` adalah otak pengambil keputusan navigasi otonom Nav2 (*Behavior Tree Navigator*). Node ini bertugas:
1. Memuat pohon perilaku XML (`BehaviorTree.CPP`), seperti pohon standar `navigate_to_pose_w_replanning_and_recovery.xml`.
2. Menerima instruksi tujuan misi dari node luar (seperti `fsm_node` atau RViz2) via Action Server `navigate_to_pose`.
3. Memanggil action client anak secara harmonis:
   - Meminta rute dari `planner_server`.
   - Mengirimkan rute ke `controller_server`.
   - Jika controller gagal/terhalang, memanggil pemulihan ke `behavior_server`.
   - Meminta rute baru jika rintangan telah bersih (*replanning loop*).

---

## 2. Antarmuka Action Server (Utama Nav2)

| Nama Action | Tipe Aksi | Deskripsi |
| :--- | :--- | :--- |
| `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Menerima goal pose dan mengeksekusi navigasi multi-tahap lengkap. |
| `navigate_through_poses` | `nav2_msgs/action/NavigateThroughPoses` | Menerima rangkaian waypoint (patroli multi-kamar). |

---

## 3. Parameter Operasi Kunci
- Frekuensi Evaluasi: `bt_loop_duration = 50 ms` (20 Hz, dioptimasi untuk Raspberry Pi 4).
- Timeout Server: `default_server_timeout = 100 ms`.
