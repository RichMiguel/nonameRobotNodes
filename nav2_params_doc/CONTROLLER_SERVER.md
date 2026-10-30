# Controller Server Configuration

File: `nav2_params_paper.yaml`  
Parent Node: `controller_server -> ros__parameters`

Controller Server bertugas menghitung perintah kecepatan kontinu (`/cmd_vel`) agar robot mengikuti lintasan (*path*) global yang telah dihasilkan oleh Planner Server. File ini mencakup pengaturan server, pemeriksa kemajuan gerak (*progress checker*), dan kriteria pencapaian tujuan (*goal checker*).

---

## Daftar Parameter Server Utama

| Parameter | Tipe | Nilai Saat Ini | Nilai Default Nav2 | Fungsi & Peran |
| :--- | :---: | :---: | :---: | :--- |
| `use_sim_time` | Boolean | `False` | `False` | Menggunakan waktu hardware nyata. |
| `controller_frequency` | Float | `10.0` | `20.0` | Frekuensi komputasi perintah gerak (Hz). Disetel ke 10 Hz agar hemat komputasi CPU Raspberry Pi 4B tanpa mengorbankan kelancaran gerak. |
| `min_x_velocity_threshold` | Float | `0.001` | `0.0001` | Ambang batas kecepatan sumbu X minimum (m/s). Kecepatan di bawah ini dianggap nol. |
| `min_y_velocity_threshold` | Float | `0.001` | `0.0001` | Ambang batas kecepatan sumbu Y minimum (m/s). |
| `min_theta_velocity_threshold` | Float | `0.001` | `0.001` | Ambang batas kecepatan sudut rotasi minimum (rad/s). |
| `failure_tolerance` | Float | `1.5` | `0.0` | Toleransi waktu kegagalan (detik) sebelum controller menyatakan status error dan memanggil recovery. |
| `progress_checker_plugin` | String | `"progress_checker"` | `"progress_checker"` | Nama instance plugin pemantau kemajuan robot. |
| `goal_checker_plugins` | Array of String | `["general_goal_checker"]` | `["general_goal_checker"]` | Nama instance plugin verifikasi tercapainya target navigasi. |
| `controller_plugins` | Array of String | `["FollowPath"]` | `["FollowPath"]` | Plugin algoritma pengikut lintasan yang digunakan (lihat dokumentasi `FOLLOWPATH.md`). |

---

## Plugin: General Goal Checker (`general_goal_checker`)

Plugin: `nav2_controller::SimpleGoalChecker`

| Parameter | Tipe | Nilai Saat Ini | Nilai Default Nav2 | Fungsi & Peran |
| :--- | :---: | :---: | :---: | :--- |
| `xy_goal_tolerance` | Float | `0.10` | `0.25` | Toleransi jarak posisi terhadap titik tujuan (meter, $10\text{ cm}$). |
| `yaw_goal_tolerance` | Float | `0.10` | `0.25` | Toleransi sudut hadap robot terhadap orientasi target (radian, $\approx 5.7^\circ$). |
| `stateful` | Boolean | `True` | `True` | Jika `True`, robot harus mengunci posisi XY terlebih dahulu sebelum mulai menyelaraskan rotasi Yaw menuju orientasi akhir. |

---

## Plugin: Progress Checker (`progress_checker`)

Plugin: `nav2_controller::SimpleProgressChecker`

| Parameter | Tipe | Nilai Saat Ini | Nilai Default Nav2 | Fungsi & Peran |
| :--- | :---: | :---: | :---: | :--- |
| `required_movement_radius` | Float | `0.2` | `0.5` | Jarak minimum perpindahan yang harus dicapai robot (meter, $20\text{ cm}$) dalam rentang waktu yang ditentukan. |
| `movement_time_allowance` | Float | `15.0` | `10.0` | Batas waktu toleransi (detik) bagi robot untuk menempuh `required_movement_radius`. Jika dalam 15 detik robot berpindah kurang dari 20 cm, robot dinyatakan macet (*stuck*). |

---

## Rasionalisasi Desain untuk Hospital Robot
1. **Presisi Titik Parkir (`xy_goal_tolerance = 0.10 m`)**: Pada rumah sakit, robot ditugaskan mengantar obat atau makanan tepat di samping ranjang pasien atau meja perawat. Toleransi 10 cm menjamin docking akurat tanpa menabrak peralatan medis.
2. **Toleransi Macet Longgar (`movement_time_allowance = 15.0 s`)**: Memberi kesempatan bagi robot untuk melambat atau berhenti sejenak ketika ada antrean orang atau kursi roda yang melintas di depannya tanpa memicu kepanikan sistem (*abrupt failure*).
