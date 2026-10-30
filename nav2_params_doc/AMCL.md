# AMCL (Adaptive Monte Carlo Localization) Configuration

File: `nav2_params_paper.yaml`  
Parent Node: `amcl -> ros__parameters`

Modul ini bertanggung jawab atas estimasi posisi probabilistik robot (*localization*) terhadap peta statis (`map`) menggunakan sensor 2D LiDAR (`/scan`) dan odometri holonomik (`/odom`).

---

## Daftar Parameter

| Parameter | Tipe | Nilai Saat Ini | Nilai Default Nav2 | Fungsi & Peran |
| :--- | :---: | :---: | :---: | :--- |
| `use_sim_time` | Boolean | `False` | `False` | Menggunakan waktu sistem hardware (bukan clock Gazebo). |
| `robot_model_type` | String | `"nav2_amcl::OmniMotionModel"` | `"nav2_amcl::DifferentialMotionModel"` | Model kinematika gerak partikel. Wajib `"nav2_amcl::OmniMotionModel"` untuk robot holonomik X-Drive (mendukung pergerakan 3-DOF: maju, mundur, strafe/geser menyamping, dan rotasi simultan). |
| `alpha1` | Float | `0.1` | `0.2` | Ekspektasi noise rotasi odometri akibat komponen rotasi robot. Nilai `0.1` disetel teredam agar partikel tidak menyebar berlebihan saat robot berputar. |
| `alpha2` | Float | `0.1` | `0.2` | Ekspektasi noise rotasi odometri akibat translasi linier robot. |
| `alpha3` | Float | `0.1` | `0.2` | Ekspektasi noise translasi linier akibat pergerakan linier robot. |
| `alpha4` | Float | `0.1` | `0.2` | Ekspektasi noise translasi linier akibat rotasi robot. |
| `alpha5` | Float | `0.1` | `0.2` | **Khusus Omni**: Ekspektasi noise translasi menyamping (*strafe / lateral slip*). |
| `recovery_alpha_slow` | Float | `0.001` | `0.0` | Laju peluruhan rata-rata lambat untuk algoritma *Kidnapped Robot Recovery* (pemulihan posisi hilang). |
| `recovery_alpha_fast` | Float | `0.10` | `0.0` | Laju peluruhan rata-rata cepat untuk memicu injeksi partikel acak saat robot tiba-tiba tersesat atau dipindahkan paksa di lorong rumah sakit. |
| `update_min_d` | Float | `0.25` | `0.2` | Jarak translasi minimum (meter) sebelum partikel AMCL di-resample. Disetel `0.25 m` untuk menghemat beban komputasi CPU Raspberry Pi 4. |
| `update_min_a` | Float | `0.20` | `0.52` | Sudut rotasi minimum (radian, $\approx 11.5^\circ$) sebelum filter diperbarui. |
| `transform_tolerance` | Float | `0.5` | `1.0` | Toleransi latensi transformasi TF (detik). Disesuaikan ke `0.5 s` untuk mengakomodasi latensi putaran RPLiDAR A1 (~148 ms) agar TF `map -> odom` tidak kadaluarsa terhadap `now()`. |
| `base_frame_id` | String | `"base_link"` | `"base_footprint"` | Frame acuan pusat robot fisik. |
| `global_frame_id` | String | `"map"` | `"map"` | Frame peta global AMCL. |
| `odom_frame_id` | String | `"odom"` | `"odom"` | Frame odometri lokal perantara. |
| `min_particles` | Integer | `500` | `500` | Batas bawah jumlah partikel saat filter konvergen (kondisi posisi stabil). |
| `max_particles` | Integer | `1500` | `2000` | Batas atas jumlah partikel saat filter tidak pasti. Dibatasi `1500` untuk mencegah lonjakan CPU 100% pada Raspberry Pi 4. |
| `scan_topic` | String | `"scan"` | `"scan"` | Topik masukan LaserScan dari LiDAR (RPLiDAR A1/A2/C1). |
| `set_initial_pose` | Boolean | `false` | `false` | Jika `false`, posisi awal robot diatur manual melalui tombol *2D Pose Estimate* di RViz2 atau via FSM. |

---

## Rasionalisasi Desain untuk Hospital Robot
1. **Pemilihan `OmniMotionModel`**: Robot rumah sakit dengan penggerak X-Drive seringkali harus berputar di tempat atau melakukan *strafe* lateral sempit. Menggunakan model diferensial akan membuat AMCL gagal melacak posisi saat robot meluncur menyamping.
2. **Noise Odometri `alpha1`–`alpha5` = 0.1**: Karena low-level firmware telah deterministik murni tanpa filter EMA dan menggunakan roda 45mm yang presisi, noise model diturunkan ke `0.1` untuk mempercepat konvergensi posisi partikel.
