# Node: map_server

Paket Asal: `nav2_map_server`  
Executable: `map_server`  
Peluncur: `nav2_bringup/localization_launch.py` (via [`robot_bringup.launch.py`](file:///home/acelz/Workspace/nonameRobot/nonameRobotNodes/src/robot_bringup/launch/robot_bringup.launch.py))  
Konfigurasi: [`nav2_params_paper.yaml`](file:///home/acelz/Workspace/nonameRobot/nonameRobotNodes/src/robot_bringup/config/nav2_params_paper.yaml) (Lihat juga: [`MAP_SERVER.md`](file:///home/acelz/Workspace/nonameRobot/nonameRobotNodes/nav2_params_doc/MAP_SERVER.md))

---

## 1. Ringkasan & Peran
`map_server` adalah node penyedia peta statis 2D (*Occupancy Grid Map Provider*). Pada robot rumah sakit ini, `map_server` bertugas:
1. Membaca metadata file peta `.yaml` (resolusi, koordinat titik origin, ambang batas occupied/free) dan file citra gambar `.pgm` atau `.png`.
2. Menyajikan representasi peta matriks 2D ke topik `/map` dengan profil QoS `Transient Local` (agar tersimpan di memori).
3. Menyediakan service `/map_server/load_map` jika sistem ingin mengganti denah ruangan secara dinamis saat runtime.

---

## 2. Antarmuka Topik & Service

### Publications (Keluaran)
| Topik | Tipe Pesan | QoS | Deskripsi |
| :--- | :--- | :--- | :--- |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Transient Local, Reliable | Matriks peta grid denah rumah sakit (0: bebas, 100: dinding, -1: unknown). |
| `/map_metadata` | `nav_msgs/msg/MapMetaData` | Transient Local, Reliable | Metadata ukuran ($W \times H$), resolusi ($0.05\text{ m/pixel}$), dan origin peta. |

### Services
| Nama Service | Tipe Service | Deskripsi |
| :--- | :--- | :--- |
| `map_server/load_map` | `nav2_msgs/srv/LoadMap` | Memuat peta baru dari path file tanpa mematikan sistem Nav2. |
