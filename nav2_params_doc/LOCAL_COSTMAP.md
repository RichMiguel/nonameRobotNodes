# Local Costmap Configuration

File: `nav2_params_paper.yaml`  
Parent Node: `local_costmap -> local_costmap -> ros__parameters`

Local Costmap menyediakan representasi grid 2D dinamis di sekitar robot (*rolling window*) untuk mendeteksi rintangan bergerak (manusia, troli, kursi roda) dan menghindari benturan secara lokal pada controller.

---

## Parameter Utama Grid & Dimensi

| Parameter | Tipe | Nilai Saat Ini | Nilai Default Nav2 | Fungsi & Peran |
| :--- | :---: | :---: | :---: | :--- |
| `update_frequency` | Float | `4.0` | `5.0` | Frekuensi pembaruan data sensor ke dalam grid costmap (Hz). Disetel ke 4 Hz untuk meringankan beban komputasi CPU Raspberry Pi 4. |
| `publish_frequency` | Float | `1.5` | `2.0` | Frekuensi visualisasi costmap ke RViz2 (Hz). 1.5 Hz menghemat bandwidth jaringan WiFi. |
| `global_frame` | String | `"odom"` | `"odom"` | Frame acuan kontinu lokal. |
| `robot_base_frame` | String | `"base_link"` | `"base_link"` | Frame posisi tengah robot. |
| `use_sim_time` | Boolean | `False` | `False` | Menggunakan waktu nyata hardware. |
| `rolling_window` | Boolean | `true` | `true` | Window peta selalu bergerak mengikuti posisi tengah robot. |
| `width` | Integer | `3` | `3` | Lebar grid costmap lokal (meter, $3\text{ m}$). |
| `height` | Integer | `3` | `3` | Panjang grid costmap lokal (meter, $3\text{ m}$). |
| `resolution` | Float | `0.05` | `0.05` | Ukuran 1 sel grid ($5\text{ cm} \times 5\text{ cm}$). Memberikan resolusi spasial detail untuk mendeteksi kaki manusia atau tiang infus. |
| `footprint` | String / Array | `'[ [0.22, 0.22], [0.22, -0.22], [-0.22, -0.22], [-0.22, 0.22] ]'` | Circular | Poligon kontur fisik robot bujursangkar $44\text{ cm} \times 44\text{ cm}$ (radius sasis $0.24\text{ m}$ / $24\text{ cm}$). |
| `plugins` | Array of String | `["obstacle_layer", "inflation_layer"]` | N/A | Lapisan aktif yang membangun costmap lokal. |

---

## Lapisan: Obstacle Layer (`obstacle_layer`)

Plugin: `nav2_costmap_2d::ObstacleLayer`

| Parameter | Tipe | Nilai Saat Ini | Nilai Default Nav2 | Fungsi & Peran |
| :--- | :---: | :---: | :---: | :--- |
| `enabled` | Boolean | `True` | `True` | Mengaktifkan pemetaan rintangan fisik secara dinamis. |
| `observation_sources` | String | `"scan"` | `"scan"` | Sumber data sensor sensorik. |
| `scan.topic` | String | `"/scan"` | `"/scan"` | Topik LaserScan dari LiDAR. |
| `scan.data_type` | String | `"LaserScan"` | `"LaserScan"` | Tipe pesan ROS 2. |
| `scan.marking` | Boolean | `True` | `True` | Menandai sel grid sebagai berpenghalang (*occupied*) saat sinar laser memantul dari objek. |
| `scan.clearing` | Boolean | `True` | `True` | Menghapus status penghalang (*free space*) jika sinar laser menembus ruang tanpa pantulan (objek telah pergi). |
| `scan.min_obstacle_height` | Float | `0.05` | `0.0` | Batas bawah ketinggian objek (meter, $5\text{ cm}$). |
| `scan.max_obstacle_height` | Float | `2.0` | `2.0` | Batas atas ketinggian objek (meter, $2\text{ m}$). |
| `scan.obstacle_min_range` | Float | `0.15` | `0.0` | Jarak pantulan minimum (meter, $15\text{ cm}$) untuk memfilter pantulan bodi robot sendiri (*self-detection*). |
| `scan.obstacle_max_range` | Float | `3.0` | `2.5` | Jarak deteksi rintangan terjauh (meter, $3\text{ m}$). |
| `scan.raytrace_min_range` | Float | `0.15` | `0.0` | Jarak minimum raytracing pembersihan. |
| `scan.raytrace_max_range` | Float | `3.5` | `3.0` | Jarak maksimum raytracing pembersihan rintangan lama ($3.5\text{ m}$). |

---

## Lapisan: Inflation Layer (`inflation_layer`)

Plugin: `nav2_costmap_2d::InflationLayer`

| Parameter | Tipe | Nilai Saat Ini | Nilai Default Nav2 | Fungsi & Peran |
| :--- | :---: | :---: | :---: | :--- |
| `inflation_radius` | Float | `0.4` | `0.55` | Radius zona penyangga bahaya di sekeliling rintangan (meter, $40\text{ cm}$). |
| `cost_scaling_factor` | Float | `4.0` | `10.0` | Faktor peluruhan kurva biaya (*exponential decay*). Nilai `4.0` memberikan transisi gradien penalti biaya yang lebar, mendorong robot menjauhi dinding secara bertahap. |

---

## Rasionalisasi Desain untuk Hospital Robot
1. **Window $3\text{ m} \times 3\text{ m}$**: Ukuran lokal ini ideal untuk lorong rumah sakit; cukup luas untuk mendeteksi orang yang berjalan dari arah berlawanan, namun cukup ringkas untuk menjaga efisiensi RAM/CPU.
2. **`inflation_radius = 0.4 m`**: Memberikan margin keamanan ekstra $16\text{ cm}$ di luar radius fisik sasis robot ($24\text{ cm}$), menjamin bodi robot tidak bersenggolan dengan dinding atau kaki ranjang pasien.
