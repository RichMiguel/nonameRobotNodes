# Node: sllidar_node

Paket: `sllidar_ros2`  
Executable: `sllidar_node`  
File Kode: [`sllidar_node.cpp`](file:///home/acelz/Workspace/nonameRobot/nonameRobotNodes/src/sllidar_ros2/src/sllidar_node.cpp)  
Peluncur: [`lidar.launch.py`](file:///home/acelz/Workspace/nonameRobot/nonameRobotNodes/src/robot_bringup/launch/lidar.launch.py)

---

## 1. Ringkasan & Peran
`sllidar_node` adalah driver perangkat keras resmi C++ untuk sensor pemindai laser 2D RPLiDAR (Slamtec). Node ini bertugas:
1. Membuka port serial hardware (misal `/dev/ttyUSB0`) dan menginisialisasi motor pemutar LiDAR.
2. Membaca data sudut dan jarak pantulan laser $360^\circ$.
3. Mengonversi data mentah menjadi pesan standar `sensor_msgs/msg/LaserScan` dan mempublikasikannya ke topik `/scan`.

---

## 2. Antarmuka Topik

### Publications (Keluaran)
| Topik | Tipe Pesan | Frekuensi Pemindaian | Deskripsi |
| :--- | :--- | :---: | :--- |
| `/scan` | `sensor_msgs/msg/LaserScan` | $10\text{–}12\text{ Hz}$ | Data jarak dan sudut $360^\circ$ di sekeliling robot (Frame ID: `laser`). |

---

## 3. Parameter Konfigurasi Kunci
| Parameter | Tipe | Nilai Tipikal | Deskripsi |
| :--- | :---: | :---: | :--- |
| `serial_port` | String | `'/dev/ttyUSB0'` | Alamat port serial hardware adapter USB-to-UART LiDAR. |
| `serial_baudrate` | Integer | `115200` atau `256000` | Baudrate serial (A1/A2M8 = 115200, A3/S1/C1 = 256000). |
| `frame_id` | String | `'laser'` | Frame ID koordinat sensor pada TF Tree. |
| `inverted` | Boolean | `false` | True jika sensor dipasang terbalik vertikal. |
| `angle_compensate`| Boolean | `true` | Kompensasi kalibrasi sudut otomatis oleh driver. |

---

## 4. Hubungan TF Tree Statis
Sensor dipasang di atas bodi robot menghadap ke belakang. Transformasi statis dideklarasikan di [`base.launch.py`](file:///home/acelz/Workspace/nonameRobot/nonameRobotNodes/src/robot_bringup/launch/base.launch.py#L37):
- $X = -0.10\text{ m}$ (mundur 10 cm dari pusat robot)
- $Y = 0.0\text{ m}$
- $Z = 0.20\text{ m}$ (ketinggian sensor)
- $\text{Yaw} = 3.14159265\text{ rad}$ ($180^\circ$, memutar arah hadap laser ke depan sasis).
- Relasi: `base_link` $\rightarrow$ `laser`.
