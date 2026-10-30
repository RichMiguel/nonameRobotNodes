# Node: odometry_node

Paket: `robot_base`  
Executable: `odometry_node`  
File Kode: [`odometry_node.py`](file:///home/acelz/Workspace/nonameRobot/nonameRobotNodes/src/robot_base/robot_base/odometry_node.py)  
Peluncur: [`base.launch.py`](file:///home/acelz/Workspace/nonameRobot/nonameRobotNodes/src/robot_bringup/launch/base.launch.py)

---

## 1. Ringkasan & Peran
`odometry_node` adalah node estimasi pergerakan posisi robot (*dead reckoning*) berbasis perangkat lunak. Node ini bertugas:
1. Menerima kecepatan aktual robot dari `communication_node`.
2. Melakukan integrasi numerik posisi $(X, Y, \theta)$ menggunakan algoritma **Runge-Kutta Orde-2 (Midpoint Integration)** untuk robot holonomik omnidirectional.
3. Mempublikasikan pesan odometri standar Nav2 pada topik `/odom`.
4. Mempublikasikan transformasi koordinat kontinu (TF) dari frame `odom` ke `base_link` secara periodik pada frekuensi stabil.

---

## 2. Antarmuka Topik & TF

### Subscriptions (Masukan)
| Topik | Tipe Pesan | Frekuensi Tipikal | Deskripsi |
| :--- | :--- | :---: | :--- |
| `raw_odom_vel` | `geometry_msgs/msg/TwistStamped` | $20\text{ Hz}$ | Kecepatan aktual sasis robot ($V_x, V_y, \omega$) dari encoder motor. |

### Publications (Keluaran)
| Topik / Saluran | Tipe Pesan | Frekuensi Tipikal | Deskripsi |
| :--- | :--- | :---: | :--- |
| `/odom` | `nav_msgs/msg/Odometry` | $20\text{ Hz}$ / $50\text{ Hz}$ | Pesan status posisi kumulatif $(X, Y, Z, \text{Quaternion})$ dan kecepatan instan lengkap dengan matriks kovariansi $6\times 6$. |
| TF Transform | `tf2_ros/TransformBroadcaster` | $20\text{ Hz}$ / $50\text{ Hz}$ | Transformasi koordinat `odom` $\rightarrow$ `base_link`. |

---

## 3. Algoritma Integrasi Holonomik Orde-2 (Midpoint Method)
Untuk mencegah akumulasi galat orientasi saat robot berputar sambil meluncur (*strafing while rotating*), digunakan sudut tengah $\theta_{\text{mid}}$:
$$\Delta \theta = \omega_z \cdot \Delta t$$
$$\theta_{\text{mid}} = \theta + \frac{\Delta \theta}{2}$$
$$\Delta X = (V_x \cos\theta_{\text{mid}} - V_y \sin\theta_{\text{mid}}) \cdot \Delta t$$
$$\Delta Y = (V_x \sin\theta_{\text{mid}} + V_y \cos\theta_{\text{mid}}) \cdot \Delta t$$
$$X \leftarrow X + \Delta X, \quad Y \leftarrow Y + \Delta Y, \quad \theta \leftarrow \theta + \Delta \theta$$

---

## 4. Parameter Konfigurasi
| Parameter | Tipe | Nilai Default | Keterangan |
| :--- | :---: | :---: | :--- |
| `odom_frequency` | Float | `20.0` | Frekuensi publikasi periodik odom & TF (Hz). |
| `odom_frame_id` | String | `'odom'` | Nama frame acuan lokal odometri. |
| `base_frame_id` | String | `'base_link'` | Nama frame acuan bodi robot. |
| `vel_timeout` | Float | `0.5` | Watchdog data serial; jika data berhenti $> 0.5\text{ s}$, kecepatan diasumsikan $0.0$. |
| `linear_scale_x` | Float | `1.0` | Faktor skala kalibrasi sumbu X (1.0 = deterministik murni). |
| `linear_scale_y` | Float | `1.0` | Faktor skala kalibrasi sumbu Y (strafe lateral). |
| `angular_scale_z` | Float | `1.0` | Faktor skala kalibrasi rotasi Yaw. |
