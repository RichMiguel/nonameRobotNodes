# Node: teleop_wasd

Paket: `robot_base`  
Executable: `teleop_wasd_node`  
File Kode: [`teleop_wasd_node.py`](file:///home/acelz/Workspace/nonameRobot/nonameRobotNodes/src/robot_base/robot_base/teleop_wasd_node.py)

---

## 1. Ringkasan & Peran
`teleop_wasd` adalah node kendali manual interaktif melalui terminal keyboard. Node ini digunakan saat:
1. Pemetaan awal lorong rumah sakit menggunakan SLAM Toolbox.
2. Pengujian respon motor dan arah putaran fisik roda.
3. Kendali manual darurat ketika navigasi otonom dinonaktifkan.

---

## 2. Pemetaan Tombol Keyboard
| Tombol | Aksi | Nilai Kecepatan yang Dikirim |
| :---: | :--- | :--- |
| **`W`** | Maju Lurus | `linear.x = 0.5 m/s, angular.z = 0.0` |
| **`S`** | Mundur Lurus | `linear.x = -0.5 m/s, angular.z = 0.0` |
| **`A`** | Putar Kiri (CCW) | `linear.x = 0.0, angular.z = 1.0 rad/s` |
| **`D`** | Putar Kanan (CW) | `linear.x = 0.0, angular.z = -1.0 rad/s` |
| *Lepas / Tombol Lain* | Berhenti | `linear.x = 0.0, angular.z = 0.0` |
| **`Ctrl + C`** | Keluar dari program | Mematikan node secara aman |

---

## 3. Antarmuka Topik

### Publications (Keluaran)
| Topik | Tipe Pesan | Frekuensi Loop | Deskripsi |
| :--- | :--- | :---: | :--- |
| `cmd_vel` | `geometry_msgs/msg/Twist` | $20\text{ Hz}$ ($0.05\text{ s}$) | Perintah kecepatan manual linier dan rotasi ke mikrokontroler. |
