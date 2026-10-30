# Node: communication_node

Paket: `robot_base`  
Executable: `communication_node`  
File Kode: [`communication_node.py`](file:///home/acelz/Workspace/nonameRobot/nonameRobotNodes/src/robot_base/robot_base/communication_node.py)  
Peluncur: [`base.launch.py`](file:///home/acelz/Workspace/nonameRobot/nonameRobotNodes/src/robot_bringup/launch/base.launch.py)

---

## 1. Ringkasan & Peran
`communication_node` adalah node jembatan perangkat keras (*hardware bridge*) antara komputer pemroses utama (Raspberry Pi 4 / PC) dan mikrokontroler penggerak roda (Arduino Mega 2560) melalui kabel USB Serial. Node ini bertugas:
1. Menerima kecepatan target robot (`/cmd_vel`) dari Nav2 / teleop, memformatnya menjadi frame biner deterministik 10-byte dengan checksum XOR, lalu mentransmisikannya ke Arduino.
2. Membaca umpan balik kecepatan roda dari Arduino secara non-blocking dengan Finite State Machine (FSM) buffer stream parser, dan mempublikasikannya sebagai `TwistStamped` pada topik `raw_odom_vel`.
3. Mengelola koneksi serial secara tangguh (*auto-reconnect*) tanpa crash saat kabel USB terlepas atau mikrokontroler di-reset.

---

## 2. Antarmuka Topik (Topic Interfaces)

### Subscriptions (Masukan)
| Topik | Tipe Pesan | Frekuensi Tipikal | Deskripsi |
| :--- | :--- | :---: | :--- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | $10\text{ Hz}$ | Kecepatan vektor target robot ($V_x, V_y, \omega$) yang dihasilkan oleh `velocity_smoother` atau `teleop_wasd`. |

### Publications (Keluaran)
| Topik | Tipe Pesan | Frekuensi Tipikal | Deskripsi |
| :--- | :--- | :---: | :--- |
| `raw_odom_vel` | `geometry_msgs/msg/TwistStamped` | $20\text{ Hz}$ | Kecepatan aktual sasis robot ($V_x, V_y, \omega$) yang diukur dari encoder motor Arduino. Frame ID: `base_link`. |

---

## 3. Protokol Komunikasi Biner

### Frame TX: Komputer $\rightarrow$ Arduino (10 Byte)
```
[Header 1: 0xAA] [Header 2: 0x55] [CMD: 0x01] [Vx_L] [Vx_H] [Vy_L] [Vy_H] [Wz_L] [Wz_H] [Checksum]
```
- Multiplier: $1000.0$ (satuan $\text{m/s}$ dan $\text{rad/s}$ dikonversi ke $\text{mm/s}$ dan $\text{mrad/s}$ pada `int16_t`).
- Checksum: XOR dari `CMD ^ Vx_L ^ Vx_H ^ Vy_L ^ Vy_H ^ Wz_L ^ Wz_H`.

### Frame RX: Arduino $\rightarrow$ Komputer (10 Byte)
```
[Header 1: 0xBB] [Header 2: 0x66] [CMD: 0x01] [Vx_L] [Vx_H] [Vy_L] [Vy_H] [Wz_L] [Wz_H] [Checksum]
```
- Parsing FSM: Menggunakan buffer dinamis (*sliding window*) untuk mendeteksi `0xBB 0x66`, memverifikasi checksum XOR, dan mengekstrak nilai tanpa blocking I/O.

---

## 4. Parameter Konfigurasi
| Parameter | Tipe | Nilai Default | Keterangan |
| :--- | :---: | :---: | :--- |
| `port` | String | `'/dev/ttyACM0'` | Alamat port serial USB Arduino Mega. |
| `baudrate` | Integer | `115200` | Kecepatan transfer serial UART. |
| `reconnect_interval` | Float | `1.0` | Interval waktu (detik) antar percobaan penyambungan ulang jika koneksi terputus. |
