# Behavior Server Configuration

File: `nav2_params_paper.yaml`  
Parent Node: `behavior_server -> ros__parameters`

Server ini mengeksekusi perilaku pemulihan (*recovery behaviors*) otomatis saat robot menemui jalan buntu, terhalang objek statis/dinamis, atau gagal menemukan jalur navigasi.

---

## Daftar Parameter

| Parameter | Tipe | Nilai Saat Ini | Nilai Default Nav2 | Fungsi & Peran |
| :--- | :---: | :---: | :---: | :--- |
| `costmap_topic` | String | `"local_costmap/costmap_raw"` | `"local_costmap/costmap_raw"` | Topik peta rintangan lokal yang digunakan untuk memeriksa tabrakan selama aksi recovery. |
| `footprint_topic` | String | `"local_costmap/published_footprint"` | `"local_costmap/published_footprint"` | Topik poligon bentuk fisik robot yang di-update costmap. |
| `cycle_frequency` | Float | `10.0` | `10.0` | Frekuensi eksekusi loop evaluasi status perilaku pemulihan (Hz). |
| `behavior_plugins` | Array of String | `["spin", "backup", "wait"]` | `["spin", "backup", "wait"]` | Daftar plugin perilaku yang didaftarkan ke server. |
| `spin.plugin` | String | `"nav2_behaviors/Spin"` | `"nav2_behaviors/Spin"` | Perilaku memutar robot di tempat untuk membersihkan rintangan di sekeliling atau mencari jalur bebas. |
| `backup.plugin` | String | `"nav2_behaviors/BackUp"` | `"nav2_behaviors/BackUp"` | Perilaku mundur perlahan dalam garis lurus saat terjebak di depan dinding atau objek penghalang. |
| `wait.plugin` | String | `"nav2_behaviors/Wait"` | `"nav2_behaviors/Wait"` | Perilaku diam sejenak menunggu manusia atau objek bergerak (misal perawat/pasien) lewat. |
| `global_frame` | String | `"odom"` | `"odom"` | Frame acuan global untuk evaluasi gerak recovery lokal. |
| `robot_base_frame` | String | `"base_link"` | `"base_link"` | Frame bodi robot. |
| `transform_tolerance` | Float | `0.5` | `0.1` | Batas toleransi keterlambatan TF transformasi (detik). Diselaraskan ke `0.5 s`. |
| `use_sim_time` | Boolean | `False` | `False` | Menggunakan waktu hardware nyata. |
| `simulate_ahead_time` | Float | `1.0` | `2.0` | Durasi simulasi proyeksi lintasan ke depan (detik) untuk memverifikasi apakah manuver recovery aman dari benturan. |
| `max_rotational_vel` | Float | `1.0` | `1.0` | Kecepatan sudut maksimum saat aksi spin (rad/s, $\approx 57.3^\circ/\text{s}$). |
| `min_rotational_vel` | Float | `0.3` | `0.4` | Kecepatan sudut minimum saat aksi spin (rad/s, $\approx 17.2^\circ/\text{s}$) untuk mengatasi inersia awal robot. |
| `rotational_acc_lim` | Float | `1.0` | `3.2` | Batas akselerasi sudut rotasi recovery (rad/s$^2$). Dibatasi `1.0` agar robot berputar halus dan tidak mengagetkan pasien/lingkungan sekitar. |

---

## Rasionalisasi Desain untuk Hospital Robot
1. **Akselerasi Rotasi Rendah (`rotational_acc_lim = 1.0`)**: Pada lingkungan rumah sakit, robot membawa perlengkapan sensitif atau sampel medis. Putaran mendadak dengan akselerasi tinggi berisiko menjatuhkan muatan dan menimbulkan suara bising.
2. **Prioritas `wait`**: Perilaku menunggu (*wait*) sangat esensial karena sebagian besar halangan di lorong rumah sakit bersifat dinamis (pasien, kursi roda, tempat tidur dorong).
