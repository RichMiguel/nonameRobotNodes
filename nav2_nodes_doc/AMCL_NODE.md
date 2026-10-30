# Node: amcl

Paket Asal: `nav2_amcl`  
Executable: `amcl`  
Peluncur: `nav2_bringup/localization_launch.py` (via [`robot_bringup.launch.py`](file:///home/acelz/Workspace/nonameRobot/nonameRobotNodes/src/robot_bringup/launch/robot_bringup.launch.py))  
Konfigurasi: [`nav2_params_paper.yaml`](file:///home/acelz/Workspace/nonameRobot/nonameRobotNodes/src/robot_bringup/config/nav2_params_paper.yaml) (Lihat juga: [`AMCL.md`](file:///home/acelz/Workspace/nonameRobot/nonameRobotNodes/nav2_params_doc/AMCL.md))

---

## 1. Ringkasan & Peran
`amcl` (Adaptive Monte Carlo Localization) adalah node pelacak posisi probabilistik 2D berbasis partikel filter. Pada robot rumah sakit ini, `amcl` bertugas:
1. Menghitung posisi terkoreksi robot terhadap denah lantai rumah sakit (`map`) dengan mencocokkan pindaian laser LiDAR (`/scan`) ke dinding pada peta Occupancy Grid.
2. Mempublikasikan transformasi TF dinamis dari `map` ke `odom` untuk mengoreksi *drift* (geseran) odometri mekanik secara kontinu.
3. Menjalankan model gerak `OmniMotionModel` untuk mendukung pergerakan omnidirectional X-Drive.

---

## 2. Antarmuka Topik & TF

### Subscriptions (Masukan)
| Topik | Tipe Pesan | Deskripsi |
| :--- | :--- | :--- |
| `/scan` | `sensor_msgs/msg/LaserScan` | Pindaian jarak laser LiDAR dari lingkungan. |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Peta denah lantai statis rumah sakit. |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Posisi awal robot yang diset oleh operator via RViz2 (*2D Pose Estimate*). |

### Publications (Keluaran)
| Topik / Saluran | Tipe Pesan | Deskripsi |
| :--- | :--- | :--- |
| `/amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Estimasi posisi dan orientasi terkini robot beserta matriks kovariansi kepastiannya. |
| `/particlecloud` | `nav2_msgs/msg/ParticleCloud` | Sebaran awan partikel hipotesis posisi robot (divisualisasikan berupa panah merah kecil di RViz2). |
| TF Transform | `tf2_ros` Broadcaster | Transformasi acuan `map` $\rightarrow$ `odom`. |

---

## 3. Parameter Operasi Kunci
- Model Gerak: `robot_model_type = "nav2_amcl::OmniMotionModel"`
- Jumlah Partikel: `min_particles = 500`, `max_particles = 1500` (dioptimasi untuk Raspberry Pi 4).
- Pembaruan: Bergerak minimal $0.25\text{ m}$ linier atau $0.20\text{ rad}$ rotasi baru melakukan resampling.
