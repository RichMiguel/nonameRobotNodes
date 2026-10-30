# Behavior Tree Navigator (BT Navigator) Configuration

File: `nav2_params_paper.yaml`  
Parent Node: `bt_navigator -> ros__parameters`

BT Navigator merupakan orkestrator inti dari Nav2. Modul ini menjalankan *Behavior Tree* XML (seperti `navigate_to_pose_w_replanning_and_recovery.xml`) untuk mengoordinasikan perencanaan jalur (planner), pelacakan jalur (controller), dan pemulihan kendala (recovery behaviors).

---

## Daftar Parameter

| Parameter | Tipe | Nilai Saat Ini | Nilai Default Nav2 | Fungsi & Peran |
| :--- | :---: | :---: | :---: | :--- |
| `use_sim_time` | Boolean | `False` | `False` | Menggunakan waktu hardware nyata. |
| `global_frame` | String | `"map"` | `"map"` | Frame peta global untuk target tujuan navigasi (*Goal Pose*). |
| `robot_base_frame` | String | `"base_link"` | `"base_link"` | Frame posisi robot yang dievaluasi. |
| `odom_topic` | String | `"/odom"` | `"/odom"` | Topik umpan balik odometri untuk pemantauan pergerakan robot secara real-time. |
| `bt_loop_duration` | Integer | `50` | `10` | Periode waktu siklus evaluasi pohon perilaku (*Behavior Tree tick interval*) dalam milidetik ($50\text{ ms} = 20\text{ Hz}$). |
| `default_server_timeout` | Integer | `100` | `20` | Batas waktu tunggu (milidetik) saat memanggil Action Server anak (planner, controller, behavior) sebelum dianggap timeout/gagal. |

---

## Rasionalisasi Desain untuk Hospital Robot
1. **Loop Duration 50 ms (20 Hz)**: Nilai default Nav2 adalah 10 ms (100 Hz), yang sangat membebani prosesor SBC seperti Raspberry Pi 4. Menurunkannya ke 50 ms (20 Hz) memangkas beban CPU secara drastis tanpa menurunkan responsivitas navigasi klinis.
2. **Server Timeout 100 ms**: Mencegah BT Navigator terburu-buru membatalkan aksi saat terjadi sedikit latensi komunikasi lokal di dalam bus ROS 2.
