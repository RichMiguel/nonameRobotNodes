# Node: voice_cmd_parser_node

Paket: `robot_voice_nav`  
Executable: `voice_cmd_parser`  
File Kode: [`voice_cmd_parser.py`](file:///home/acelz/Workspace/nonameRobot/nonameRobotNodes/src/robot_voice_nav/robot_voice_nav/voice_cmd_parser.py)  
Peluncur: [`voice.launch.py`](file:///home/acelz/Workspace/nonameRobot/nonameRobotNodes/src/robot_bringup/launch/voice.launch.py)

---

## 1. Ringkasan & Peran
`voice_cmd_parser_node` adalah jembatan antarmuka suara (*Voice / NLU Interface*) antara modul pemroses bahasa alami (seperti pada paket `comand-inst`) dan sistem navigasi robot. Node ini bertugas:
1. Membuka server socket TCP pada `0.0.0.0:5000` di dalam *thread background* terpisah.
2. Menerima string JSON berisi intent suara dan entitas lokasi tujuan.
3. Memetakan nama lokasi (misal `"ruang icu"`, `"kamar 203"`) ke koordinat $X, Y, \text{Yaw}$ berdasarkan file konfigurasi [`waypoints.yaml`](file:///home/acelz/Workspace/nonameRobot/nonameRobotNodes/src/robot_voice_nav/waypoints.yaml).
4. Mempublikasikan target posisi ke topik `/goal_pose` untuk dieksekusi oleh FSM atau Nav2.

---

## 2. Format Protokol Socket TCP (JSON Payload)

Klien (misal GUI / mic listener di laptop atau Raspberry Pi) mengirimkan string JSON:
```json
{
  "intent": "NAVIGATE",
  "slots": {
    "location": "ruang_icu"
  }
}
```
Jika `intent` adalah `"STOP"`, node menerima perintah pembatalan darurat.

---

## 3. Antarmuka Topik

### Publications (Keluaran)
| Topik | Tipe Pesan | Deskripsi |
| :--- | :--- | :--- |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | Target navigasi akhir pada koordinat peta global (frame: `map`). |

---

## 4. Konfigurasi Waypoints ([`waypoints.yaml`](file:///home/acelz/Workspace/nonameRobot/nonameRobotNodes/src/robot_voice_nav/waypoints.yaml))
Contoh pemetaan titik kamar rumah sakit:
```yaml
ruang_icu:
  x: 2.45
  y: 1.80
  yaw: 0.0
kamar_203:
  x: 5.20
  y: -1.40
  yaw: 1.57
lobi_utama:
  x: 0.00
  y: 0.00
  yaw: 0.0
```
