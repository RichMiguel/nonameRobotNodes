# Map Server Configuration

File: `nav2_params_paper.yaml`  
Parent Node: `map_server -> ros__parameters`

Map Server bertugas memuat peta statis 2D Occupancy Grid (file `.yaml` dan gambar peta `.pgm`/`.png`) dari disk dan menyediakannya ke sistem Nav2 melalui service dan topik `/map`.

---

## Daftar Parameter

| Parameter | Tipe | Nilai Saat Ini | Nilai Default Nav2 | Fungsi & Peran |
| :--- | :---: | :---: | :---: | :--- |
| `use_sim_time` | Boolean | `False` | `False` | Menggunakan waktu nyata hardware. |
| `yaml_filename` | String | `""` | `""` | Path ke file konfigurasi peta `.yaml`. Nilai string kosong ini disuplai secara dinamis saat runtime melalui launch argument `map:=<path/to/map.yaml>` pada file launch `robot_bringup.launch.py`. |

---

## Rasionalisasi Desain untuk Hospital Robot
1. **Dynamic Injection via Launch Argument**: Peta rumah sakit dapat dengan mudah ditukar (misal antara `lab_map_new.yaml`, `lantai_1.yaml`, atau `bangsal_melati.yaml`) langsung dari baris perintah atau menu `start_robot.sh` tanpa perlu mengedit file parameter utama.
