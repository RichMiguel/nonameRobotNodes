# FollowPath Controller Plugin (Regulated Pure Pursuit)

File: `nav2_params_paper.yaml`  
Parent Node: `controller_server -> ros__parameters -> FollowPath`  
Plugin: `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`  
Referensi Desain: *Tabel 1 Eksperimen Paper arXiv:2601.15006*

Plugin ini adalah implementasi algoritma **Regulated Pure Pursuit (RPP)** yang dirancang khusus untuk robot logistik/layanan dalam ruangan guna menghasilkan pergerakan halus, bebas osilasi (*anti-zigzag*), dan aman saat menikung maupun saat berada di dekat rintangan.

---

## Daftar Parameter

### 1. Kecepatan & Adaptasi Jarak Lookahead
| Parameter | Tipe | Nilai Saat Ini | Nilai Default Nav2 | Fungsi & Peran |
| :--- | :---: | :---: | :---: | :--- |
| `desired_linear_vel` | Float | `0.2` | `0.5` | Kecepatan jelajah linier nominal ($0.2\text{ m/s}$ atau $720\text{ m/jam}$). Kecepatan tenang dan aman untuk navigasi di koridor rumah sakit. |
| `use_velocity_scaled_lookahead_dist` | Boolean | `true` | `false` | Menyesuaikan jarak *lookahead* secara dinamis proporsional terhadap kecepatan saat ini ($L = v \times t_{\text{lookahead}}$). |
| `lookahead_time` | Float | `1.0` | `1.5` | Faktor pengali waktu adaptasi lookahead (detik). |
| `lookahead_dist` | Float | `0.75` | `0.6` | Jarak acuan lookahead nominal (meter). Disetel `0.75 m` untuk meredam osilasi lateral. |
| `min_lookahead_dist` | Float | `0.55` | `0.3` | Batas bawah jarak lookahead (meter). Mencegah robot bereaksi terlalu agresif pada belokan tajam. |
| `max_lookahead_dist` | Float | `0.90` | `0.9` | Batas atas jarak lookahead (meter). Mencegah robot mengambil jalan pintas memotong sudut dinding (*corner-cutting*). |

---

### 2. Regulasi Kecepatan Berdasarkan Kurvatur (Kelengkungan Tikungan)
| Parameter | Tipe | Nilai Saat Ini | Nilai Default Nav2 | Fungsi & Peran |
| :--- | :---: | :---: | :---: | :--- |
| `use_regulated_linear_velocity_scaling` | Boolean | `true` | `true` | Mengaktifkan perlambatan otomatis saat robot mendeteksi kurvatur jalur yang tajam. |
| `regulated_linear_scaling_min_radius` | Float | `0.90` | `0.90` | Radius kurvatur batas (meter). Jika radius tikungan $< 0.90\text{ m}$, robot otomatis memperlambat kecepatan liniernya. |
| `regulated_linear_scaling_min_speed` | Float | `0.25` | `0.25` | Batas kecepatan terendah saat deselerasi kurvatur aktif (m/s). |

---

### 3. Regulasi Pendekatan Tujuan (Approach Scaling)
| Parameter | Tipe | Nilai Saat Ini | Nilai Default Nav2 | Fungsi & Peran |
| :--- | :---: | :---: | :---: | :--- |
| `approach_velocity_scaling_dist` | Float | `0.60` | `0.60` | Jarak dari target akhir (meter, $60\text{ cm}$) di mana robot mulai melakukan deselerasi bertahap (*soft braking*). |
| `min_approach_linear_velocity` | Float | `0.05` | `0.05` | Kecepatan rayap terendah ($5\text{ cm/s}$) saat mendekati titik docking target akhir. |

---

### 4. Regulasi Berdasarkan Kedekatan Halangan (Cost Heuristic)
| Parameter | Tipe | Nilai Saat Ini | Nilai Default Nav2 | Fungsi & Peran |
| :--- | :---: | :---: | :---: | :--- |
| `use_cost_regulated_linear_velocity_scaling` | Boolean | `true` | `true` | Mengurangi kecepatan robot secara otomatis jika jarak bodi robot ke dinding/halangan semakin sempit. |
| `cost_scaling_dist` | Float | `0.40` | `0.60` | Jarak ambang kedekatan (meter, $40\text{ cm}$) di mana deselerasi mulai diterapkan. |
| `cost_scaling_gain` | Float | `1.0` | `1.0` | Faktor bobot pengali deselerasi berbasis costmap. |

---

### 5. Penyelarasan Arah Hadap (Rotate to Heading)
| Parameter | Tipe | Nilai Saat Ini | Nilai Default Nav2 | Fungsi & Peran |
| :--- | :---: | :---: | :---: | :--- |
| `use_rotate_to_heading` | Boolean | `true` | `false` | Memaksa robot berputar di tempat menghadap arah jalur terlebih dahulu sebelum melaju maju. |
| `rotate_to_heading_min_angle` | Float | `0.8` | `0.785` | Selisih sudut minimum (radian, $\approx 45.8^\circ$) untuk memicu aksi putar di tempat. |
| `rotate_to_heading_angular_vel` | Float | `0.6` | `1.8` | Kecepatan sudut saat berputar di tempat (rad/s, $\approx 34.4^\circ/\text{s}$). Dibuat tenang agar tidak memicu distorsi pembacaan laser LiDAR. |
| `max_angular_accel` | Float | `0.6` | `3.2` | Batas akselerasi sudut rotasi (rad/s$^2$). Mencegah sentakan torsi motor pada roda omni saat mulai berputar. |

---

### 6. Keselamatan & Pencegahan Tabrakan
| Parameter | Tipe | Nilai Saat Ini | Nilai Default Nav2 | Fungsi & Peran |
| :--- | :---: | :---: | :---: | :--- |
| `use_collision_detection` | Boolean | `true` | `true` | Melakukan verifikasi tabrakan real-time di sepanjang busur lintasan proyeksi RPP. |
| `max_allowed_time_to_collision_up_to_carrot` | Float | `0.45` | `1.0` | Waktu prediksi tabrakan ke titik target (*carrot point*) dalam detik. Jika diprediksi akan menabrak dalam $< 0.45\text{ s}$, robot langsung berhenti darurat. |
| `transform_tolerance` | Float | `0.5` | `0.1` | Batas toleransi TF transformasi (detik). Disesuaikan ke `0.5 s` agar proses lookup transform koordinat global tidak mengalami timeout / wait-blocking. |
| `inflation_cost_scaling_factor` | Float | `5.0` | `3.0` | Skala kurva penurunan biaya inflasi untuk verifikasi tabrakan. |

---

## Rasionalisasi Desain untuk Hospital Robot
1. **Peredaman Osilasi (*Anti-Zigzag*)**: Nilai `lookahead_dist = 0.75 m` (dengan batas bawah $0.55\text{ m}$) memfilter koreksi kemudi berlebih yang sering menimbulkan lintasan zig-zag saat menyusuri lorong panjang.
2. **Kenyamanan Pasien & Keamanan**: Akselerasi sudut dibatasi sangat halus (`max_angular_accel = 0.6 rad/s²` dan `rotate_to_heading_angular_vel = 0.6 rad/s`), menjamin robot bergerak tenang, elegan, dan tidak membuat kaget orang di sekitarnya.
