# Velocity Smoother Configuration

File: `nav2_params_paper.yaml`  
Parent Node: `velocity_smoother -> ros__parameters`  
Referensi Desain: *Tabel 1 Paper arXiv:2601.15006*

Velocity Smoother bertindak sebagai filter akselerasi/deselerasi dan batas kecepatan (*speed & acceleration limiter*) pada tahap akhir sebelum perintah `/cmd_vel` dikirimkan ke hardware robot (`communication_node`).

---

## Daftar Parameter

| Parameter | Tipe | Nilai Saat Ini | Nilai Default Nav2 | Fungsi & Peran |
| :--- | :---: | :---: | :---: | :--- |
| `use_sim_time` | Boolean | `False` | `False` | Menggunakan waktu nyata hardware. |
| `smoothing_frequency` | Float | `10.0` | `20.0` | Frekuensi pembentukan profil kecepatan (Hz). Selaras dengan `controller_frequency` (10 Hz). |
| `scale_velocities` | Boolean | `True` | `False` | Jika batas akselerasi atau deselerasi salah satu sumbu terlampaui, seluruh komponen vektor kecepatan diskalakan secara proporsional untuk menjaga arah lintasan. |
| `feedback` | String | `"OPEN_LOOP"` | `"OPEN_LOOP"` | Menghitung akselerasi berdasarkan perintah sebelumnya (*open-loop*), mencegah getaran feedback akibat noise odometri. |
| `max_velocity` | Array of Float (3) | `[0.30, 0.01, 1.0]` | `[0.5, 0.0, 1.0]` | Batas kecepatan tertinggi $[V_x, V_y, \omega]$: Maju maks $0.30\text{ m/s}$, geser lateral $0.01\text{ m/s}$, rotasi $1.0\text{ rad/s}$. |
| `min_velocity` | Array of Float (3) | `[-0.30, -0.01, -1.0]` | `[-0.5, 0.0, -1.0]` | Batas kecepatan terendah/mundur $[V_x, V_y, \omega]$: Mundur maks $0.30\text{ m/s}$, geser kanan $0.01\text{ m/s}$, rotasi CW $-1.0\text{ rad/s}$. |
| `max_accel` | Array of Float (3) | `[0.30, 0.01, 1.0]` | `[2.5, 0.0, 3.2]` | Batas akselerasi positif $[a_x, a_y, \alpha]$: Translasi $0.30\text{ m/s}^2$, rotasi $1.0\text{ rad/s}^2$. |
| `max_decel` | Array of Float (3) | `[-0.30, -0.01, -1.0]` | `[-2.5, 0.0, -3.2]` | Batas deselerasi/pengereman $[d_x, d_y, \alpha]$: Translasi $-0.30\text{ m/s}^2$, rotasi $-1.0\text{ rad/s}^2$. |
| `odom_topic` | String | `"odom"` | `"odom"` | Topik umpan balik odometri. |
| `odom_duration` | Float | `0.1` | `0.1` | Rentang waktu rata-rata pengukuran odometri (detik). |

---

## Analisis Komponen Sumbu $Y$ (Lateral)
Pada konfigurasi paper ini, komponen $V_y$ dibatasi pada angka kecil (`[-0.01, 0.01]` m/s). Hal ini sengaja diatur karena controller utama menggunakan **Regulated Pure Pursuit (RPP)** yang berfokus pada pelacakan arah hadap (*forward heading tracking*). Pembatasan $V_y$ ini mencegah terjadinya selip lateral roda omni saat robot bermanuver di lantai rumah sakit yang licin/berkeramik.

---

## Rasionalisasi Desain untuk Hospital Robot
1. **Akselerasi Lembut ($0.30\text{ m/s}^2$)**: Memberikan kenyamanan maksimal dan mencegah tumpahan cairan obat atau guncangan pada perangkat medis yang dibawa robot.
2. **Deselerasi Terkendali ($-0.30\text{ m/s}^2$)**: Menghindari pengereman mendadak yang dapat mengikis roller roda omniwheel serta memicu bahaya tersandung bagi pasien di sekitar robot.
