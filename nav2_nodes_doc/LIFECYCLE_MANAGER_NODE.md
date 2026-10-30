# Node: lifecycle_manager

Paket Asal: `nav2_lifecycle_manager`  
Executable: `lifecycle_manager`  
Peluncur: `nav2_bringup/localization_launch.py` & `navigation_launch.py` (via [`robot_bringup.launch.py`](file:///home/acelz/Workspace/nonameRobot/nonameRobotNodes/src/robot_bringup/launch/robot_bringup.launch.py))

---

## 1. Ringkasan & Peran
`lifecycle_manager` adalah pengelola status kesiapan (*ROS 2 Managed Lifecycle State Manager*) untuk seluruh server Nav2. Node ini bertugas:
1. Memastikan seluruh node Nav2 diinisialisasi dalam urutan deterministik:
   `Unconfigured` $\rightarrow$ `Configuring` $\rightarrow$ `Inactive` $\rightarrow$ `Activating` $\rightarrow$ `Active`.
2. Mencegah kondisi balapan (*race condition*) di mana Controller atau Planner mulai berjalan sebelum Peta (`map_server`) atau Transformasi TF siap.
3. Memantau kesehatan node-node Nav2 secara terus-menerus (*bond timer*). Jika salah satu node penting mati (misal controller crash), lifecycle manager dapat mematikan (*transition to inactive*) seluruh sistem demi keselamatan hardware.

---

## 2. Node-Node yang Dikelola pada Proyek Ini

### Instance 1: `lifecycle_manager_localization`
Mengatur siklus hidup subsistem lokalisasi:
- `map_server`
- `amcl`

### Instance 2: `lifecycle_manager_navigation`
Mengatur siklus hidup subsistem navigasi jalur:
- `controller_server`
- `smoother_server` / `velocity_smoother`
- `planner_server`
- `behavior_server`
- `bt_navigator`

---

## 3. Parameter Konfigurasi Kunci
- `autostart = true`: Otomatis mengonfigurasi dan mengaktifkan seluruh node saat peluncuran (*launch*) tanpa perlu pemicu manual service CLI.
- `bond_timeout = 4.0`: Toleransi waktu deteksi denyut jantung (*heartbeat*) antar node (detik).
