# Node: velocity_smoother

Paket Asal: `nav2_velocity_smoother`  
Executable: `velocity_smoother`  
Peluncur: `nav2_bringup/navigation_launch.py` (via [`robot_bringup.launch.py`](file:///home/acelz/Workspace/nonameRobot/nonameRobotNodes/src/robot_bringup/launch/robot_bringup.launch.py))  
Konfigurasi: [`nav2_params_paper.yaml`](file:///home/acelz/Workspace/nonameRobot/nonameRobotNodes/src/robot_bringup/config/nav2_params_paper.yaml) (Lihat juga: [`VELOCITY_SMOOTHER.md`](file:///home/acelz/Workspace/nonameRobot/nonameRobotNodes/nav2_params_doc/VELOCITY_SMOOTHER.md))

---

## 1. Ringkasan & Peran
`velocity_smoother` adalah node pembatas profil akselerasi, deselerasi, dan kecepatan (*Kinodynamic Limiter*). Node ini bertindak sebagai pintu gerbang terakhir perintah Nav2 sebelum diteruskan ke driver mikrokontroler. Fungsinya:
1. Menerima kecepatan kasar (*raw cmd_vel*) dari `controller_server`.
2. Menghaluskan perubahan kecepatan secara bertahap sesuai batas akselerasi ($a_{\text{max}} = 0.30\text{ m/s}^2$) dan deselerasi ($-0.30\text{ m/s}^2$).
3. Mencegah sentakan mekanis (*jerk*), slip roda omni pada lantai rumah sakit, dan tumpahan cairan obat yang diangkut robot.
4. Mempublikasikan output kecepatan final ke topik resmi `/cmd_vel` yang di-subscribe oleh `communication_node`.

---

## 2. Antarmuka Topik

### Subscriptions (Masukan)
| Topik | Tipe Pesan | Deskripsi |
| :--- | :--- | :--- |
| `cmd_vel_nav` (atau `cmd_vel_raw`) | `geometry_msgs/msg/Twist` | Perintah kecepatan mentah dari controller RPP. |
| `/odom` | `nav_msgs/msg/Odometry` | Umpan balik kecepatan aktual saat ini. |

### Publications (Keluaran)
| Topik | Tipe Pesan | Frekuensi Loop | Deskripsi |
| :--- | :--- | :---: | :--- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | $10\text{ Hz}$ | Perintah kecepatan yang telah dibatasi dan dihaluskan menuju hardware Arduino. |

---

## 3. Parameter Operasi Kunci
- Kecepatan Maksimum: $[V_x = 0.30\text{ m/s}, V_y = 0.01\text{ m/s}, \omega = 1.0\text{ rad/s}]$.
- Akselerasi / Deselerasi Maksimum: $0.30\text{ m/s}^2$ translasi dan $1.0\text{ rad/s}^2$ rotasi.
- Proporsionalitas: `scale_velocities = True` (menjaga arah vektor gerak jika salah satu sumbu melampaui akselerasi).
