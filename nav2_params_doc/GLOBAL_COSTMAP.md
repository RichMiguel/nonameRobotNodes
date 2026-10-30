# Global Costmap Configuration

File: `nav2_params_paper.yaml`  
Parent Node: `global_costmap -> global_costmap -> ros__parameters`

Global Costmap mencakup seluruh denah lantai rumah sakit (berdasarkan file peta static map `.yaml`) digabungkan dengan rintangan sensorik terkini. Peta ini digunakan oleh Planner Server untuk mencari rute global terpendek dan teraman dari posisi saat ini menuju ruangan tujuan.

---

## Parameter Utama Grid & Peta

| Parameter | Tipe | Nilai Saat Ini | Nilai Default Nav2 | Fungsi & Peran |
| :--- | :---: | :---: | :---: | :--- |
| `update_frequency` | Float | `1.0` | `1.0` | Frekuensi pembaruan grid global (Hz). Cukup 1 Hz karena peta denah bersifat statis. |
| `publish_frequency` | Float | `1.0` | `1.0` | Frekuensi publikasi ke RViz2 (Hz). |
| `global_frame` | String | `"map"` | `"map"` | Frame acuan global denah rumah sakit. |
| `robot_base_frame` | String | `"base_link"` | `"base_link"` | Frame posisi tengah robot. |
| `use_sim_time` | Boolean | `False` | `False` | Menggunakan waktu nyata hardware. |
| `resolution` | Float | `0.05` | `0.05` | Resolusi grid global ($5\text{ cm}$ per sel). |
| `track_unknown_space` | Boolean | `true` | `false` | Membedakan area yang belum dijelajahi (*unknown/grey space*) dari area bebas (*free space*). Sangat penting agar planner tidak merencanakan rute menembus dinding yang belum terpetakan. |
| `footprint` | String / Array | `'[ [0.22, 0.22], [0.22, -0.22], [-0.22, -0.22], [-0.22, 0.22] ]'` | Circular | Poligon dimensi robot $44\text{ cm} \times 44\text{ cm}$. |
| `plugins` | Array of String | `["static_layer", "obstacle_layer", "inflation_layer"]` | N/A | Tiga lapisan pembentuk costmap global. |

---

## Lapisan: Static Layer (`static_layer`)

Plugin: `nav2_costmap_2d::StaticLayer`

| Parameter | Tipe | Nilai Saat Ini | Nilai Default Nav2 | Fungsi & Peran |
| :--- | :---: | :---: | :---: | :--- |
| `map_subscribe_transient_local` | Boolean | `True` | `True` | Menjaga pesan peta tetap tersimpan di memori QoS ROS 2 sehingga node baru yang aktif langsung menerima peta tanpa menunggu publikasi ulang. |

---

## Lapisan: Obstacle Layer (`obstacle_layer`)

Plugin: `nav2_costmap_2d::ObstacleLayer`

| Parameter | Tipe | Nilai Saat Ini | Nilai Default Nav2 | Fungsi & Peran |
| :--- | :---: | :---: | :---: | :--- |
| `enabled` | Boolean | `True` | `True` | Memasukkan rintangan LiDAR ke peta global sehingga planner global dapat merencanakan jalan memutar jika ada lorong yang terblokir. |
| `observation_sources` | String | `"scan"` | `"scan"` | Sumber data LiDAR. |
| `scan.topic` | String | `"/scan"` | `"/scan"` | Topik LaserScan. |
| `scan.data_type` | String | `"LaserScan"` | `"LaserScan"` | Tipe pesan. |
| `scan.marking` | Boolean | `True` | `True` | Menandai rintangan baru pada peta global. |
| `scan.clearing` | Boolean | `True` | `True` | Menghapus rintangan yang sudah hilang. |
| `scan.obstacle_min_range` | Float | `0.15` | `0.0` | Batas filter bodi robot ($15\text{ cm}$). |
| `scan.obstacle_max_range` | Float | `3.0` | `2.5` | Jarak deteksi ($3\text{ m}$). |
| `scan.raytrace_min_range` | Float | `0.15` | `0.0` | Jarak raytracing minimum ($15\text{ cm}$). |
| `scan.raytrace_max_range` | Float | `3.5` | `3.0` | Jarak raytracing pembersihan ($3.5\text{ m}$). |

---

## Lapisan: Inflation Layer (`inflation_layer`)

Plugin: `nav2_costmap_2d::InflationLayer`

| Parameter | Tipe | Nilai Saat Ini | Nilai Default Nav2 | Fungsi & Peran |
| :--- | :---: | :---: | :---: | :--- |
| `inflation_radius` | Float | `0.55` | `0.55` | Radius zona inflasi global ($55\text{ cm}$). |
| `cost_scaling_factor` | Float | `5.0` | `10.0` | Faktor gradien biaya. Nilai `5.0` mendorong planner global memilih rute yang berada di tengah-tengah lorong rumah sakit (jauh dari dinding). |

---

## Rasionalisasi Desain untuk Hospital Robot
1. **`inflation_radius = 0.55 m` & `cost_scaling_factor = 5.0`**: Menghasilkan *global path* yang selalu memandu robot menyusuri bagian tengah koridor rumah sakit, mencegah robot berjalan terlalu mepet ke pintu kamar pasien atau sudut lorong yang rawan orang keluar mendadak.
2. **`track_unknown_space = true`**: Menjamin rute navigasi hanya dirancang melewati area rumah sakit yang benar-benar telah diverifikasi aman.
